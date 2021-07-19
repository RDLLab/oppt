/** @file LinkedHashSet.hpp
 *
 * Contains the LinkedHashSet class, which stores a set that keeps ordering between elements.
 */
#ifndef SOLVER_LINKEDHASHSET_HPP_
#define SOLVER_LINKEDHASHSET_HPP_

#include <iostream>
#include <initializer_list>
#include <list>
#include <unordered_map>

namespace oppt {
/** LinkedHashSet
 *
 * Represents a set of elements which maintains iteration in the order of insertion.
 * - Insertion / deletion in constant average time complexity (using std::unordered_map)
 */
template <typename Element>
class LinkedHashSet {
  public:
    /** Constructs an empty LinkedHashSet. */
    LinkedHashSet() :
        map_(),
        elementList_() {
    }

    /** Constructs a LinkedHashSet with the given elements, in the order they occur. */
    LinkedHashSet(std::initializer_list<Element> elements) :
        LinkedHashSet() {
        for (Element e : elements) {
            add(e);
        }
    }

    /** Constructs a LinkedHashSet with the elements from the given iterator range,
     * in the order they occur.
     */
    template <typename InputIterator>
    LinkedHashSet(InputIterator first, InputIterator last) :
        LinkedHashSet() {
        for (auto it = first; it != last; it++) {
            add(*it);
        }
    }

    ~LinkedHashSet() = default;

    /** A const_iterator begin() for easy iteration. */
    typename std::list<Element>::const_iterator begin() const {
        return elementList_.cbegin();
    }

    /** A const_iterator end() for easy iteration. */
    typename std::list<Element>::const_iterator end() const {
        return elementList_.cend();
    }

    /** Clears this set. */
    void clear() {
        map_.clear();
        elementList_.clear();
    }

    /** Returns the number of elements in this set. */
    long size() const {
        return elementList_.size();
    }

    /** If it is not already present, adds the given element to this set in the final position. */
    void add(Element entry) {
        if (contains(entry)) {
            return;
        }
        elementList_.push_back(entry);
        map_.emplace(entry, std::prev(elementList_.end()));
    }

    /** If it is present, removes the given element from this set. */
    void remove(Element entry) {
        if (!contains(entry)) {
            return;
        }
        elementList_.erase(map_[entry]);
        map_.erase(entry);
    }

    /** Returns the current first element of this set.*/
    Element getFirst() const {
        return elementList_.front();
    }

    /** Returns true iff this set contains the given element. */
    bool contains(Element entry) const {
        return map_.count(entry) > 0;
    }

  private:
    /** An unordered_map for quick lookup of an iterator for each element. */
    std::unordered_map<Element, typename std::list<Element>::iterator> map_;
    /** A linked list to store the elements in order. */
    std::list<Element> elementList_;
};
} /* namespace oppt */

#endif /* SOLVER_LINKEDHASHSET_HPP_ */
