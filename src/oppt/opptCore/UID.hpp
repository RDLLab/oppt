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
#ifndef _OPPT_UID_HPP_
#define _OPPT_UID_HPP_
#include <atomic>

namespace oppt
{
namespace UID
{

/**
 * @brief Generates a unique ID
 */
inline size_t getUniqueId() {
	static std::atomic<size_t> uid {0};
	return uid++;    
}

} // end UID
} // end oppt

#endif
