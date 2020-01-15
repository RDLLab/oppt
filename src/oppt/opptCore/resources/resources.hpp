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
#ifndef __OPPT_RESOURCES_HPP__
#define __OPPT_RESOURCES_HPP__
//#include <experimental/filesystem>
#include "oppt/opptCore/logging.hpp"
#include "oppt/opptCore/utils.hpp"
#include "oppt/opptCore/typedefs.hpp"

namespace oppt
{
namespace resources
{

using namespace oppt;
namespace fs = boost::filesystem;

inline bool FolderExists(const std::string &folderName) {
    char const* tmp = getenv("OPPT_RESOURCE_PATH");
    std::string uri = "";
    if (tmp == NULL) {
        return false;
    }

    std::string opptEnv(tmp);
    VectorString paths;
    split(opptEnv, ':', paths);
    for (auto & path : paths) {
        if (path != "") {
            fs::directory_iterator end;
            for (fs::directory_iterator pathIter(path); pathIter != end; pathIter++) {
                VectorString elems;
                split(pathIter->path().string(), "/", elems);
                if (elems[elems.size() - 1] == folderName)
                    return true;
            }
        }
    }

    return false;
}

inline std::string FindFolder(const std::string &folderName) {
    char const* tmp = getenv("OPPT_RESOURCE_PATH");
    std::string uri = "";
    if (tmp == NULL) {
        return uri;
    }

    std::string opptEnv(tmp);
    VectorString paths;
    split(opptEnv, ':', paths);
    for (auto & path : paths) {
        if (path != "") {
            fs::directory_iterator end;
            for (fs::directory_iterator pathIter(path); pathIter != end; pathIter++) {
                VectorString elems;
                split(pathIter->path().string(), "/", elems);
                if (elems[elems.size() - 1] == folderName)
                    return pathIter->path().string();
            }
        }
    }

    return "";
}

inline std::string FindFile(const std::string& filename)
{
    //Check for absolute path first
    if (fs::exists(filename)) {
        return filename;
    }

    char const* tmp = getenv("OPPT_RESOURCE_PATH");
    std::string uri = "";
    if (tmp == NULL) {
        return uri;
    }

    std::string opptEnv(tmp);
    VectorString paths;
    split(opptEnv, ':', paths);
    for (auto & path : paths) {
        if (path != "") {
            if (filename.find("file://") != std::string::npos) {
                std::string filenameCut = filename;
                filenameCut.erase(0, 6);
                std::string fullPath = path + filenameCut;
                if (fs::exists(fullPath)) {
                    return fullPath;
                }
            } else if (filename.find("model://") != std::string::npos) {
                std::string filenameCut = filename;
                filenameCut.erase(0, 7);
                std::string fullPath = path + filenameCut;
                if (fs::exists(fullPath)) {
                    return fullPath;
                }
            }
            else {
                fs::directory_iterator end;
                for (fs::directory_iterator pathIter(path); pathIter != end; pathIter++) {
                    if (fs::is_directory(pathIter->path())) {
                        fs::directory_iterator endFile;
                        for (fs::directory_iterator fileIter(pathIter->path()); fileIter != endFile; fileIter++) {
                            if (fs::is_regular_file(fileIter->path())) {
                                VectorString fileElems;
                                split(fileIter->path().string(), '/', fileElems);
                                if (fileElems[fileElems.size() - 1] == filename) {
                                    std::string filepath = fileIter->path().string();
                                    return filepath;
                                }
                            }
                        }
                    }
                }
            }
        }
    }


    return uri;
}

inline bool FileExists(const std::string& filename)
{
    //Check for absolute path first
    if (fs::exists(filename)) {
        return true;
    }

    char const* tmp = getenv("OPPT_RESOURCE_PATH");
    if (tmp != NULL) {
        std::string opptEnv(tmp);
        VectorString paths;
        split(opptEnv, ':', paths);
        for (auto & path : paths) {
            if (path != "") {
                if (filename.find("file://") != std::string::npos) {
                    std::string filenameCut = filename;
                    filenameCut.erase(0, 6);
                    std::string fullPath = path + filenameCut;
                    if (fs::exists(fullPath)) {
                        return true;
                    }

                } else if (filename.find("model://") != std::string::npos) {
                    std::string filenameCut = filename;
                    filenameCut.erase(0, 7);
                    std::string fullPath = path + filenameCut;
                    if (fs::exists(fullPath)) {
                        return true;
                    }
                } else {
                    fs::directory_iterator end;
                    for (fs::directory_iterator pathIter(path); pathIter != end; pathIter++) {
                        if (fs::is_directory(pathIter->path())) {
                            fs::directory_iterator endFile;
                            for (fs::directory_iterator fileIter(pathIter->path()); fileIter != endFile; fileIter++) {
                                if (fs::is_regular_file(fileIter->path())) {
                                    VectorString fileElems;
                                    split(fileIter->path().string(), '/', fileElems);
                                    if (fileElems[fileElems.size() - 1] == filename) {
                                        return true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}


}
}

#endif

