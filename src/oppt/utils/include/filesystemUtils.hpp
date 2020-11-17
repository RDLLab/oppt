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
#ifndef __FILESYSTEM_UTILS_HPP__
#define __FILESYSTEM_UTILS_HPP__
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

using std::cout;
using std::endl;

namespace oppt
{

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::stringstream ss;
    ss << std::setprecision(n) << a_value;
    return ss.str();
}

/**
 * @brief Check if a given file (full path) exists
 */
inline bool fileExists(const std::string& filename)
{
    return boost::filesystem::exists(filename);
}

/**
 * @brief Creates a directory
 */
inline bool createDir(const std::string& path)
{
    if (boost::filesystem::is_directory(path)) {
        return true;
    } else {
        boost::filesystem::path dir(path);
        return boost::filesystem::create_directories(dir);
    }

    return false;
}

// Removes all files inside dir
inline bool clearDirectory(const std::string& path)
{
    return boost::filesystem::remove_all(path);
}

/**
 * @brief Removes a file
 */
inline bool removeFile(const std::string& path)
{    
    bool removed = boost::filesystem::remove(path);    
    return removed;
}

/**
 * @brief Copies a source path to a destination path
 * 
 * @param sourcePath The source path
 * @param destinationPath The destination path
 * @param removeExisting Remove existing destination path
 */
inline bool copyFile(const std::string& sourcePath,
              const std::string& destinationPath,
              bool removeExisting = true)
{
    if (removeExisting) {
        removeFile(destinationPath);
    }
    std::ifstream  src(sourcePath, std::ios::binary);
    std::ofstream  dst(destinationPath, std::ios::binary);
    dst << src.rdbuf();
    return true;
}

}

#endif
