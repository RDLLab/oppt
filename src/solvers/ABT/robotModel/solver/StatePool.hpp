#ifndef _ABT_STATE_POOL_HPP_
#define _ABT_STATE_POOL_HPP_
#include "solvers/ABT/solver/StatePool.hpp"
#include "oppt/opptCore/typedefs.hpp"

namespace abt
{
    
class OPPTStatePool: public StatePool
{
    friend class Solver;
    friend class TextSerializer;
public:
    OPPTStatePool(std::unique_ptr<StateIndex> stateIndex);
    
    virtual ~OPPTStatePool();

    virtual StateInfo* createOrGetInfo(std::unique_ptr<State> state) override;
    
    virtual StateInfo *getInfo(State const *state) const override;
    
    virtual StateInfo *getInfoById(long id) const override;
    
    virtual long getNumberOfStates() const override;
    
    virtual std::vector<State const*> getStates() const override;
    
    void cleanup();
    
    struct Hash {
        /** Returns the hash value for the pointed-to state. */
        std::size_t operator()(State const *state) const {
            return state->hash();
        }
    };
    
    struct EqualityTest {
        /** Returns true if the two pointed-to stated are equal. */
        bool operator()(State const *s1, State const *s2) const {
            return *s1 == *s2;
        }
    };
    
    typedef std::unordered_map<State const*, std::unique_ptr<StateInfo>, Hash, EqualityTest> StateInfosMap;
    
    StateInfosMap::const_iterator begin() const {
        return stateInfosMap_.begin();
    }
    
    StateInfosMap::const_iterator end() const {
        return stateInfosMap_.end();
    }
private:
    
    StateInfosMap stateInfosMap_;
};

}

#endif
