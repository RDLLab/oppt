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
#ifndef _COMMAND_TYPES_HPP_
#define _COMMAND_TYPES_HPP_

namespace oppt
{

struct DisplayCommand {
    std::string typeString = "";

};

struct AddDisplayCommand: public DisplayCommand {
    AddDisplayCommand():
        DisplayCommand() {
        typeString = "addDisplay";

    }

    std::string name = "";

    std::string displayType = "";

    bool enabled = true;

    std::string topic = "";

    std::string topicType = "";
    
    std::string frame = "";

};

struct RemoveDisplayCommand: public DisplayCommand {
    RemoveDisplayCommand():
        DisplayCommand() {
        typeString = "removeDisplay";

    }

    std::string name = "";
};

struct SetFixedFrameCommand: public DisplayCommand {
    SetFixedFrameCommand():
        DisplayCommand() {
        typeString = "setFixedFrame";

    }

    std::string fixedFrame = "";
};

struct SetFrameRateCommand: public DisplayCommand {
    SetFrameRateCommand():
        DisplayCommand() {
            typeString = "setFrameRate";
    }

    unsigned int frameRate = 0;

};

struct TakeScreenshotCommand: public DisplayCommand {
    TakeScreenshotCommand(): DisplayCommand() {
        typeString = "takeScreenshot";
    }

    std::string filename = "";

};

struct AddDisplayProperties {
    std::string command = "";

    std::string name = "";

    std::string displayType = "";

    bool enabled = true;

    std::string topic = "";

    std::string topicType = "";

    bool setFixedFrame = false;

    std::string fixedFrame = "";

};

}

#endif
