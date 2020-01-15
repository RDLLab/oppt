/** @file Vector.cpp
 *
 * Provides default implementations for some basic methods of the vector class in terms of the
 * asVector() method.
 */
#include "Vector.hpp"

#include <cmath>
#include <cstddef>

#include <vector>

#include "oppt/global.hpp"

namespace abt {

double Vector::distanceTo(Point const &otherPoint) const {
    std::vector<double> v1 = this->asVector();
    std::vector<double> v2 = static_cast<Vector const *>(
            &otherPoint)->asVector();
    std::vector<double>::iterator i1 = v1.begin();
    std::vector<double>::iterator i2 = v2.begin();
    double distanceSquared = 0;
    for (; i1 < v1.end(); i1++, i2++) {
        distanceSquared += std::pow(*i1 - *i2, 2);
    }
    return std::sqrt(distanceSquared);
}

bool Vector::equals(Point const &otherPoint) const {
    std::vector<double> v1 = this->asVector();
    std::vector<double> v2 = static_cast<Vector const *>(
            &otherPoint)->asVector();
    std::vector<double>::iterator i1 = v1.begin();
    std::vector<double>::iterator i2 = v2.begin();

    for (; i1 < v1.end(); i1++, i2++) {
        if (*i1 != *i2) {
            return false;
        }
    }
    return true;
}

std::size_t Vector::hash() const {
    std::vector<double> v1 = this->asVector();

    std::size_t hashValue = 0;
    for (std::vector<double>::iterator i1 = v1.begin(); i1 < v1.end(); i1++) {
        oppt::hash_combine(hashValue, *i1);
    }
    return hashValue;

}

void Vector::print(std::ostream &os) const {
    std::vector<double> values = asVector();
    os << "(";
    for (std::vector<double>::const_iterator it = values.begin();
            it < values.end(); it++) {
        os << *it;
        if (it + 1 != values.end()) {
            os << ",";
        }
    }
    os << ")";
}

} /* namespace abt */
