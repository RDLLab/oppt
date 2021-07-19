#ifndef __CONTINUOUS_ROBOT_ACTION_CONTAINER_HPP__
#define __CONTINUOUS_ROBOT_ACTION_CONTAINER_HPP__
#include "RobotAction.hpp"
#include "oppt/robotHeaders/ActionSpace.hpp"

namespace shared
{

class ContinuousRobotActionContainer: public abt::ContinuousActionContainerBase
{
public:
    typedef robot::ContActionConstructionData KeyType;
    typedef typename KeyType::HashEqualOptions HashEqualOptions;
private:
    /** Service class so the unordered_map can access hash() and equal(). */
    class Comparator
    {
        HashEqualOptions options;
    public:
        Comparator(const HashEqualOptions& theOptions): options(theOptions) {};

        size_t operator()(const KeyType& key) const {
            return key.hash(options);
        }
        size_t operator()(const KeyType& first, const KeyType& second) const {
            return first.equal(second, options);
        }
    };

public:
    /**ContinuousRobotActionContainer(const HashEqualOptions& options): container(0, Comparator(options), Comparator(options)) {
    }*/

    ContinuousRobotActionContainer(const HashEqualOptions& options, size_t& dim);
    
    virtual ~ContinuousRobotActionContainer() {}

    _NO_COPY_OR_MOVE(ContinuousRobotActionContainer);

    virtual size_t size() const override;

    virtual std::unique_ptr<abt::ContinuousActionMapEntry>& at(const abt::ContinuousActionConstructionDataBase& key) override;

    virtual const std::unique_ptr<abt::ContinuousActionMapEntry>& at(const abt::ContinuousActionConstructionDataBase& key) const override;

    virtual std::unique_ptr<abt::ContinuousActionMapEntry>& operator[](const abt::ContinuousActionConstructionDataBase& key) override;

    virtual std::unique_ptr<abt::ContinuousActionMapEntry>& at(const FloatType* constructionDataVector) override;

    virtual const std::unique_ptr<abt::ContinuousActionMapEntry>& at(const FloatType* constructionDataVector) const override;

    virtual std::unique_ptr<abt::ContinuousActionMapEntry>& operator[](const FloatType* constructionDataVector) override;

    virtual void clear() override;

    virtual std::vector<abt::ActionMappingEntry const*> getEntries() const override;

    virtual std::vector<abt::ActionMappingEntry const*> getEntriesWithChildren() const override;

    virtual std::vector<abt::ActionMappingEntry const*> getEntriesWithNonzeroVisitCount() const override;
    
private:
    std::unordered_map<KeyType, std::unique_ptr<abt::ContinuousActionMapEntry>, Comparator, Comparator> container;

    size_t dimensions_;
};

class ContActionPool final: public abt::ContinuousActionPool
{
public:
    ContActionPool(oppt::ActionSpaceSharedPtr &actionSpace);

    virtual ~ContActionPool() = default;

    _NO_COPY_OR_MOVE(ContActionPool);


    /** Returns a container to store actions within a ContinuousActionMap */
    virtual std::unique_ptr<ContinuousActionContainerBase> createActionContainer(BeliefNode* node) const override;

    /** Returns an action construction data object based on a vector of numbers that was provided.
     *
     * Here, constructionData is a pointer to a data array as it is returned by
     * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
     * create new actions based on values it seems fit.
     */
    virtual std::unique_ptr<ContinuousActionConstructionDataBase> createActionConstructionData(const FloatType* constructionDataVector, const BeliefNode* belief) const override;

    /** Returns an action based on the Construction Data that was provided.
     *
     * In this version, constructionData is a pointer to a data array as it is returned by
     * ContinuousActionConstructionDataBase::data(). It enables the action chooser to
     * create new actions based on values it seems fit.
     *
     * The default version uses createActionConstructionData first and then creates an action based
     * on the full construction data. This might be inefficient and an implementation can override
     * this function for a more direct approach.
     *
     * TODO: Check whether this function is actually used or can be removed.
     */
    virtual std::unique_ptr<Action> createAction(const FloatType* constructionDataVector, const BeliefNode* belief) const override;


    /** Returns an action based on the Construction Data that was provided.
     *
     * The default version calls createAction(constructionData.data()) which is probably fine
     * in a purely continuous case, but probably not in a hybrid case.
     */
    virtual std::unique_ptr<Action> createAction(const ContinuousActionConstructionDataBase& constructionData) const override;


    /** Returns the initial bounding box for the continuous search.
     *
     * For each dimension, the first entry of the pair is the lower bound, the second entry is the upper bound.
     */
    virtual std::vector<std::pair<FloatType, FloatType>> getInitialBoundingBox(BeliefNode* belief) const override;

 
    /** Returns a shared pointer to a container containing the construction data for the additional fixed actions in a hybrid action space.
     *
     * The result is a shared pointer. Thus, the implementation can decide whether it wants to create the container and pass on ownership or it
     * can return a reference to an internal vector without having to re-create it every time.
     *
     * The default version returns null to indicate there are no fixed actions.
     */
    virtual std::vector<std::unique_ptr<ContinuousActionConstructionDataBase>> createFixedActions(const BeliefNode* belief) const override;


private:
    std::vector<std::pair<FloatType, FloatType>> initial_bounding_box_;    

    oppt::ActionSpaceSharedPtr actionSpace_;
};

}

#endif
