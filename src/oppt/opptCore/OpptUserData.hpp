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
#ifndef _OPPT_USER_DATA_
#define _OPPT_USER_DATA_
#include "oppt/opptCore/includes.hpp"

namespace oppt
{

/**
 * Base class for any UserData object
 */
class OpptUserData
{
public:
    _NO_COPY_BUT_MOVE(OpptUserData);

    /** @brief Default constructor*/
    OpptUserData() = default;

    /** @brief Default virtual destructor */
    virtual ~OpptUserData() = default;

    /**
     * @brief Static cast to type T
     * @tparam T The action type to cast to
     */
    _STATIC_CAST

    /** @brief Serialize to an output stream */
    virtual void serialize(std::ostream& os, const std::string &prefix="") const { }

    /** @brief Print to an output stream */
    virtual void print(std::ostream& os) const {}

    friend std::ostream& operator<< (std::ostream& out, const OpptUserData& opptUserData) {
        opptUserData.print(out);
        return out;
    }
};

/** @brief std::shared_ptr to oppt::OpptUserData*/
typedef std::shared_ptr<OpptUserData> OpptUserDataSharedPtr;

}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
