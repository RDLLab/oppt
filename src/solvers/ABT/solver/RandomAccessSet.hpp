/** @file RandomAccessSet.hpp
 *
 * Contains the RandomAccessSet class, which represents a basic set with random access.
 */
#ifndef SOLVER_RANDOMACCESSSET_HPP_
#define SOLVER_RANDOMACCESSSET_HPP_

#include <iostream>
#include <initializer_list>
#include <unordered_map>
#include <vector>

namespace oppt {
/** RandomAccessSet
 *
 * Represents a set of elements:
 * - Access by index in O(1) time (using std::vector)
 * - Insertion / deletion in constant average time complexity (using std::unordered_map)
 */
template <typename Element>
class RandomAccessSet {
  public:
    /** Constructs an empty RandomAccessSet. */
    RandomAccessSet() :
        map_(),
        elementList_() {
    }

    /** Constructs a RandomAccessSet with the given elements. */
    RandomAccessSet(std::initializer_list<Element> elements) :
        RandomAccessSet() {
        for (Element e : elements) {
            add(e);
        }
    }
    ~RandomAccessSet() = default;

    /** A const_iterator begin() for easy iteration. */
    typename std::vector<Element>::const_iterator begin() const {
        return elementList_.cbegin();
    }

    /** A const_iterator end() for easy iteration. */
    typename std::vector<Element>::const_iterator end() const {
        return elementList_.cend();
    }

    /** Resets this set. */
    void clear() {
        map_.clear();
        elementList_.clear();
    }

    /** Returns the number of elements in this set. */
    long size() const {
        return elementList_.size();
    }

    /** Adds the given element to this set, if it is not already present. */
    bool add(Element entry) {
        if (contains(entry)) {
            return false;
        }
        elementList_.push_back(entry);
        auto ret = map_.emplace(entry, elementList_.size() - 1);
	return true;
    }

    /** Removes the given element from this set if it is present. */
    bool remove(Element entry) {
        if (!contains(entry)) {
            return false;
        }
        long index = map_[entry];
        long lastIndex = elementList_.size() - 1;

        if (index != lastIndex) {
            Element lastEntry = elementList_[lastIndex];
            elementList_[index] = lastEntry;
            map_[lastEntry] = index;
        }

        // Remove extraneous elements.
        elementList_.pop_back();
        map_.erase(entry);
	return true;
    }

    /** Returns the element at the given index. */
    Element get(long index) const {
        return elementList_[index];
    }

    /** Returns the index of the given element, or -1 if it is not present. */
    int indexOf(Element entry) const {
        if (!contains(entry)) {
            return -1;
        }
        return map_[entry];
    }

    /** Returns true iff this map contains the given element. */
    bool contains(Element entry) const {
        return map_.count(entry) > 0;
    }

  private:
    /** An unordered_map for quick lookup of the index of each element. */
    std::unordered_map<Element, long> map_;
    /** A vector to store the elements. */
    std::vector<Element> elementList_;
};
} /* namespace oppt */

#endif /* SOLVER_RANDOMACCESSSET_HPP_ */
