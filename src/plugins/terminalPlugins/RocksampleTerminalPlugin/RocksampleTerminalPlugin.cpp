/**
 * Copyright 2017
 * 
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License published by the Free Software Foundation, 
 * either version 2 of the License, or (at your option) any later version.
 * 
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OPPT. 
 * If not, see http://www.gnu.org/licenses/.
 */
#include "oppt/plugin/Plugin.hpp"

namespace oppt
{
class RocksampleTerminalPlugin: public TerminalPlugin
{
public:
    RocksampleTerminalPlugin():
        TerminalPlugin() {
    }

    virtual ~RocksampleTerminalPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
        ValidityReportSharedPtr vr(new ValidityReport(propagationResult->nextState));        
        VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();
        vr->isValid = true;
        if (stateVec[0] < 0.5 || stateVec[0] > 8.5 || stateVec[1] < 0.5 || stateVec[1] > 7.5)
            vr->isValid = false;
        return vr;
    }

    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {
        //return false;
        if (!isValid(propagationResult)->isValid)
            return true;        
        VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();

        // Check if we reached the right-hand side of the map
        if (stateVec[0] > 7.5)
            return true;
        return false;
    }
    
};

OPPT_REGISTER_TERMINAL_PLUGIN(RocksampleTerminalPlugin)
}
