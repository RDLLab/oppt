#ifndef __ROBOT_UTILS_
#define __ROBOT_UTILS_
#include <vector>
#include <string>
#include <iostream>

using std::cout;
using std::endl;

namespace robot
{

template<typename T>
void printVector(const std::vector<T>& vec, std::string str)
{
    cout << str << ": ";
    for (auto & k : vec) {
        cout << k << ", ";
    }

    cout << endl;
};

}

#endif
