#include "ContinuousRobotActionContainer.hpp"

using namespace oppt;

namespace shared
{

ContinuousRobotActionContainer::ContinuousRobotActionContainer(const HashEqualOptions& options, size_t& dim):
    container(0, Comparator(options), Comparator(options)),
    dimensions_(dim)
{

}

size_t ContinuousRobotActionContainer::size() const
{
    return container.size();
}

std::unique_ptr< abt::ContinuousActionMapEntry >& ContinuousRobotActionContainer::at(const abt::ContinuousActionConstructionDataBase& key)
{   
    return container.at(static_cast<const KeyType&>(key));
}

const std::unique_ptr< abt::ContinuousActionMapEntry >& ContinuousRobotActionContainer::at(const abt::ContinuousActionConstructionDataBase& key) const
{
    return container.at(static_cast<const KeyType&>(key));
}

std::unique_ptr< abt::ContinuousActionMapEntry >& ContinuousRobotActionContainer::operator[](const abt::ContinuousActionConstructionDataBase& key)
{    
    return container[static_cast<const KeyType&>(key)];
}

std::unique_ptr< abt::ContinuousActionMapEntry >& ContinuousRobotActionContainer::at(const FloatType* constructionDataVector)
{
    return container.at(static_cast<const KeyType&>(KeyType(constructionDataVector, dimensions_)));
}

const std::unique_ptr< abt::ContinuousActionMapEntry >& ContinuousRobotActionContainer::at(const FloatType* constructionDataVector) const
{
    return container.at(static_cast<const KeyType&>(KeyType(constructionDataVector, dimensions_)));
}

std::unique_ptr< abt::ContinuousActionMapEntry >& ContinuousRobotActionContainer::operator[](const FloatType* constructionDataVector)
{    
    return container[static_cast<const KeyType&>(KeyType(constructionDataVector, dimensions_))];
}

void ContinuousRobotActionContainer::clear()
{
    container.clear();
}

std::vector<abt::ActionMappingEntry const*> ContinuousRobotActionContainer::getEntries() const
{    
    std::vector<abt::ActionMappingEntry const*> result;
    result.reserve(container.size());
    for (auto & i : container) {
        result.push_back(i.second.get());
    }
    return std::move(result);
}

std::vector<abt::ActionMappingEntry const*> ContinuousRobotActionContainer::getEntriesWithChildren() const
{
    std::vector<abt::ActionMappingEntry const*> result;
    // let's assume most of them have children.
    result.reserve(container.size());
    for (auto & i : container) {
        abt::ContinuousActionMapEntry const& entry = *(i.second);
        if (entry.getChild() != nullptr) {
            result.push_back(&entry);
        }
    }
    return std::move(result);
}

std::vector<abt::ActionMappingEntry const*> ContinuousRobotActionContainer::getEntriesWithNonzeroVisitCount() const
{
    std::vector<abt::ActionMappingEntry const*> result;
    // let's assume most of them have been visited.
    result.reserve(container.size());
    for (auto & i : container) {
        abt::ContinuousActionMapEntry const& entry = *(i.second);
        if (entry.getVisitCount() > 0) {
            if (!entry.isLegal()) {
                debug::show_message("WARNING: Illegal entry with nonzero visit count!");
            }
            result.push_back(&entry);
        }
    }
    return std::move(result);
}


ContActionPool::ContActionPool(oppt::ActionSpaceSharedPtr& actionSpace):
    initial_bounding_box_(),
    actionSpace_(actionSpace)
{
    VectorFloat lowerLimits;
    VectorFloat upperLimits;
    actionSpace_->getActionLimits()->getLimits()->as<VectorLimitsContainer>()->get(lowerLimits, upperLimits);
    initial_bounding_box_ = std::vector<std::pair<FloatType, FloatType>>(lowerLimits.size());
    for (size_t i = 0; i < lowerLimits.size(); i++) {
        initial_bounding_box_[i] = std::pair<FloatType, FloatType>(lowerLimits[i], upperLimits[i]);
    }   
}

std::unique_ptr<ContActionPool::ContinuousActionContainerBase> ContActionPool::createActionContainer(BeliefNode* /*node*/) const
{
    size_t dim = actionSpace_->getNumDimensions();
    return std::make_unique<shared::ContinuousRobotActionContainer>(robot::ContActionConstructionData::HashEqualOptions(0.03333333333333), dim);
}

std::unique_ptr<ContActionPool::ContinuousActionConstructionDataBase> ContActionPool::createActionConstructionData(const FloatType* constructionDataVector, const BeliefNode* /*belief*/) const
{    
    size_t dataSize = actionSpace_->getNumDimensions();
    return std::make_unique<robot::ContActionConstructionData>(constructionDataVector, dataSize);
}

std::unique_ptr<abt::Action> ContActionPool::createAction(const FloatType* constructionDataVector, const BeliefNode* /*belief*/) const
{
    size_t dataSize = actionSpace_->getNumDimensions();    
    return std::make_unique<robot::ContinuousRobotAction>(constructionDataVector, dataSize);
}

std::unique_ptr<abt::Action> ContActionPool::createAction(const ContinuousActionConstructionDataBase& constructionData) const
{    
    const FloatType* constructionDataVector = static_cast<const robot::ContActionConstructionData&>(constructionData).data();    
    size_t dataSize = static_cast<const robot::ContActionConstructionData&>(constructionData).size();
    return std::make_unique<robot::ContinuousRobotAction>(constructionDataVector, dataSize);
}

std::vector<std::pair<FloatType, FloatType>> ContActionPool::getInitialBoundingBox(BeliefNode* /*belief*/) const
{
    return initial_bounding_box_;
}

std::vector<std::unique_ptr<ContActionPool::ContinuousActionConstructionDataBase>> ContActionPool::createFixedActions(const BeliefNode* /*belief*/) const
{
    std::vector<std::unique_ptr<ContActionPool::ContinuousActionConstructionDataBase>> result;
    return result;
}
}
