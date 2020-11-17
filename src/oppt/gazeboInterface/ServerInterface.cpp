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
#include "ServerInterface.hpp"
#include <string>

namespace oppt {

//gazebo::Master* g_master = NULL;
std::vector<gazebo::SystemPluginPtr> g_plugins;

bool ServerInterface::setup(const std::string& _prefix, int _argc, char** _argv,
                            std::vector<gazebo::SystemPluginPtr>& _plugins)
{
    gazebo::common::load();

    // The SDF find file callback.
    sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

    // Initialize the informational logger. This will log warnings, and
    // errors.
    gzLogInit(_prefix, "default.log");

    // Load all the system plugins
    for (std::vector<gazebo::SystemPluginPtr>::iterator iter =
                _plugins.begin(); iter != _plugins.end(); ++iter) {
        (*iter)->Load(_argc, _argv);
    }

    if (!gazebo::transport::init()) {
        gzerr << "Unable to initialize transport.\n";
        return false;
    }

    // Make sure the model database has started.
    gazebo::common::ModelDatabase::Instance()->Start();

    // Run transport loop. Starts a thread
    gazebo::transport::run();

    // Init all system plugins
    for (std::vector<gazebo::SystemPluginPtr>::iterator iter = _plugins.begin();
            iter != _plugins.end(); ++iter) {
        (*iter)->Init();
    }

    return true;
}

bool ServerInterface::setupServer(int _argc, char** _argv, const unsigned int &inc)
{
    std::string host = "";
    unsigned int port = 0;

    gazebo::transport::get_master_uri(host, port);
    port += inc;

    master_ = std::unique_ptr<gazebo::Master>(new gazebo::Master());
    master_->Init(port);
    master_->RunThread();

    //g_master = new gazebo::Master();
    //g_master->Init(port);
    //g_master->RunThread();

    if (!setup("server-", _argc, _argv, g_plugins)) {
        gzerr << "Unable to setup Gazebo\n";
        return false;
    }

    if (!gazebo::sensors::load()) {
        gzerr << "Unable to load sensors\n";
        return false;
    }

    if (!gazebo::physics::load()) {
        gzerr << "Unable to initialize physics.\n";
        return false;
    }

    if (!gazebo::sensors::init()) {
        gzerr << "Unable to initialize sensors\n";
        return false;
    }

    return true;
}

void ServerInterface::stop() {
    if (master_)
        master_->Fini();
}

}

