/** @file DiscretizedPoint.cpp
 *
 * Contains the implementation of the DiscretizedPoint class.
 */
#include "DiscretizedPoint.hpp"

#include "Point.hpp"

namespace abt {
bool DiscretizedPoint::equals(Point const &otherPoint) const {
    DiscretizedPoint const &other =
            static_cast<DiscretizedPoint const &>(otherPoint);
    return getBinNumber() == other.getBinNumber();
}

std::size_t DiscretizedPoint::hash() const {
    return getBinNumber();
}

void DiscretizedPoint::print(std::ostream &os) const {
    os << getBinNumber();
}
} /* namespace abt */
