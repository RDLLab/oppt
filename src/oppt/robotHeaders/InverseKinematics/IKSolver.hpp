#ifndef _OPPT_IK_SOLVER_HPP_
#define _OPPT_IK_SOLVER_HPP_
#include "oppt/opptCore/core.hpp"
#include "oppt/global.hpp"

namespace oppt {
class IKSolver {
public:
	IKSolver(RandomGenerator* randGen,
	         const std::string &urdfFile,
	         const std::string &baseLink,
	         const std::string &tipLink);

	virtual ~IKSolver();

	_NO_COPY_BUT_MOVE(IKSolver)

	_STATIC_CAST	

	virtual std::pair<bool, VectorFloat> solve(const geometric::Pose &pose,
	        const unsigned int &numAttempts,
	        const VectorFloat &qInit = VectorFloat()) = 0;

	virtual std::unique_ptr<IKSolver> clone() const = 0;

protected:
	RandomGenerator* randGen_ = nullptr;

	std::string urdfFile_;

	std::string baseLink_;

	std::string tipLink_;

};

class DefaultIKSolver: public IKSolver {
public:
	DefaultIKSolver(RandomGenerator* randGen,
	                const std::string &urdfFile,
	                const std::string &baseLink,
	                const std::string &tipLink);

	virtual ~DefaultIKSolver();

	virtual std::pair<bool, VectorFloat> solve(const geometric::Pose &pose,
	        const unsigned int &numAttempts,
	        const VectorFloat &qInit = VectorFloat()) override;

	virtual std::unique_ptr<IKSolver> clone() const override;
};

typedef std::unique_ptr<IKSolver> IKSolverUniquePtr;

}

#endif