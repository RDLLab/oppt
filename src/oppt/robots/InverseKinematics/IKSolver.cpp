#include "oppt/robotHeaders/InverseKinematics/IKSolver.hpp"
#include "oppt/opptCore/resources/resources.hpp"

namespace oppt {

IKSolver::IKSolver(RandomGenerator* randGen,
                   const std::string &urdfFile,
                   const std::string &baseLink,
                   const std::string &tipLink):
	randGen_(randGen),
	urdfFile_(urdfFile),
	baseLink_(baseLink),
	tipLink_(tipLink) {
	if (!(urdfFile.empty())) {
		if (!oppt::resources::FileExists(urdfFile))
			ERROR("urdf file '" + urdfFile + "' doesn't exist");
		std::string urdfPath = oppt::resources::FindFile(urdfFile);
		urdfFile_ = urdfPath;
	}
}

IKSolver::~IKSolver() {

}

DefaultIKSolver::DefaultIKSolver(RandomGenerator* randGen,
                                 const std::string &urdfFile,
                                 const std::string &baseLink,
                                 const std::string &tipLink):
	IKSolver(randGen, urdfFile, baseLink, tipLink) {

}

DefaultIKSolver::~DefaultIKSolver() {

}

std::pair<bool, VectorFloat> DefaultIKSolver::solve(const geometric::Pose &pose,
        const unsigned int &numAttempts,
        const VectorFloat &qInit) {
	return std::make_pair(false, VectorFloat());
}

std::unique_ptr<IKSolver> DefaultIKSolver::clone() const  {
	IKSolverUniquePtr cloned(new DefaultIKSolver(randGen_, urdfFile_, baseLink_, tipLink_));
	return std::move(cloned);
}

}