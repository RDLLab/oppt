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
#ifndef _OPPT_SERVER_INTERFACE_HPP_
#define _OPPT_SERVER_INTERFACE_HPP_
#include <oppt/opptCore/core.hpp>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>

namespace oppt {
class ServerInterface {
public:
	bool setupServer(int _argc, char** _argv, const unsigned int &inc);

	void stop();

private:
	std::unique_ptr<gazebo::Master> master_ = nullptr;

private:
	bool setup(const std::string& _prefix, int _argc, char** _argv,
	           std::vector<gazebo::SystemPluginPtr>& _plugins);
};
}

#endif
