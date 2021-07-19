#include "oppt/robotHeaders/InverseKinematics/InverseKinematics.hpp"
#include <urdf_parser/urdf_parser.h>

using namespace oppt;

TracIKSolver::TracIKSolver(RandomGenerator* randGen,
                           const std::string &urdfFile,
                           const std::string &baseLink,
                           const std::string &tipLink):
	IKSolver(randGen, urdfFile, baseLink, tipLink) {
}

bool TracIKSolver::init() {
	urdf::ModelInterfaceSharedPtr robotModel = urdf::parseURDFFile(urdfFile_);
	kdlTree_ = std::make_unique<KDL::Tree>();
	if (!(kdl_parser::treeFromUrdfModel(*(robotModel.get()), *(kdlTree_.get())))) {
		cout << "KDL Tree could not be initialized" << endl;
		return false;
	}
	chain_ = std::make_unique<KDL::Chain>();
	if (!(kdlTree_->getChain(baseLink_, tipLink_, *(chain_.get())))) {
		cout << "KDL chain could not be initialized with base link '" + baseLink_ + "' and tip link '" + tipLink_ + "'" << endl;
		return false;
	}

	q_.resize(chain_->getNrOfJoints());

	//inverseJacobianSolver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(*(chain_.get()));
	lb.resize(chain_->getNrOfJoints());
	ub.resize(chain_->getNrOfJoints());
	std::vector<KDL::Segment> chainSegs = chain_->segments;
	urdf::JointConstSharedPtr joint;
	uint jointNum = 0;
	for (unsigned int i = 0; i != chainSegs.size(); ++i) {
		joint = robotModel->getJoint(chainSegs[i].getJoint().getName());
		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
			jointNum++;
			float lower, upper;
			bool hasLimits = false;
			if ( joint->type != urdf::Joint::CONTINUOUS ) {
				if (joint->safety) {
					if (inFixedJoints(chainSegs[i].getJoint().getName())) {
						auto jv = fixedJoints_.at(chainSegs[i].getJoint().getName());
						fixedJointAngles_[jointNum - 1] = jv;
						lower = jv;
						upper = jv;
					} else {
						lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
						upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
					}
				} else {
					if (inFixedJoints(chainSegs[i].getJoint().getName())) {
						auto jv = fixedJoints_.at(chainSegs[i].getJoint().getName());
						fixedJointAngles_[jointNum - 1] = jv;
						lower = jv;
						upper = jv;
					} else {
						lower = joint->limits->lower;
						upper = joint->limits->upper;
					}
				}
				hasLimits = true;
			}

			if (hasLimits) {
				lb(jointNum - 1) = lower;
				ub(jointNum - 1) = upper;
			}
			else {
				lb(jointNum - 1) = std::numeric_limits<float>::lowest();
				ub(jointNum - 1) = std::numeric_limits<float>::max();
			}
		}
	}

	fixedJointsChain_ = std::make_unique<KDL::Chain>();
	for (size_t i = 0; i != chainSegs.size(); ++i) {
		if (inFixedJoints(chainSegs[i].getJoint().getName())) {
			KDL::Joint fixedJoint(chainSegs[i].getJoint().getName(), KDL::Joint::JointType::None);
			KDL::Segment segment(chainSegs[i].getName(), fixedJoint, chainSegs[i].getFrameToTip());
			fixedJointsChain_->addSegment(segment);
		} else {
			KDL::Segment segment(chainSegs[i].getName(), chainSegs[i].getJoint(), chainSegs[i].getFrameToTip());
			fixedJointsChain_->addSegment(segment);
		}
	}

	jacobianSolver_ = std::make_unique<KDL::ChainJntToJacSolver>(*(fixedJointsChain_.get()));
	jacobian_ = std::make_unique<KDL::Jacobian>(fixedJointsChain_->getNrOfJoints());

	inverseJacobianSolver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(*(fixedJointsChain_.get()));

	FloatType timeout = 0.005;
	FloatType eps = 1e-3;

	ikSolver_ = std::make_unique<TRAC_IK::TRAC_IK>(*(chain_.get()), lb, ub, timeout, eps);
	initialized_ = true;
	LOGGING("Initialized TracIK");
	return true;
}

geometric::Pose TracIKSolver::endEffectorPoseFromJointAngles(const VectorFloat &jointAngles) {
	KDL::ChainFkSolverPos_recursive forwardSolver(*(chain_.get()));
	KDL::JntArray q;
	q.resize(chain_->getNrOfJoints());
	unsigned int idx = 0;
	for (size_t i = 0; i != chain_->getNrOfJoints(); ++i) {
		if (fixedJointAngles_.find(i) != fixedJointAngles_.end()) {
			q(i) = fixedJointAngles_.at(i);
		} else {
			q(i) = jointAngles[idx];
			idx++;
		}
	}

	KDL::Frame result;
	forwardSolver.JntToCart(q, result);

	geometric::Pose res;
	res.position.x() = result.p.x();
	res.position.y() = result.p.y();
	res.position.z() = result.p.z();
	FloatType w, x, y, z;
	result.M.GetQuaternion(x, y, z, w);
	res.orientation.w() = w;
	res.orientation.x() = x;
	res.orientation.y() = y;
	res.orientation.z() = z;
	return res;

}

std::pair<bool, VectorFloat> TracIKSolver::solve(const geometric::Pose &pose,
        const unsigned int &numAttempts,
        const VectorFloat &qInit) {
	if (!initialized_)
		return std::make_pair(false, VectorFloat());
	KDL::Vector translation(pose.position.x(), pose.position.y(), pose.position.z());
	auto rotation = KDL::Rotation::Quaternion(pose.orientation.x(),
	                pose.orientation.y(),
	                pose.orientation.z(),
	                pose.orientation.w());
	KDL::Frame kdlPose(rotation, translation);
	unsigned int numActuatedJoints = chain_->getNrOfJoints() - fixedJointAngles_.size();
	KDL::JntArray qInit_, result;
	qInit_.resize(chain_->getNrOfJoints());
	for (size_t i = 0; i != numAttempts; ++i) {
		if (qInit.size() == numActuatedJoints) {
			unsigned int idx = 0;
			for (size_t j = 0; j != chain_->getNrOfJoints(); ++j) {
				if (fixedJointAngles_.find(j) != fixedJointAngles_.end()) {
					qInit_(j) = fixedJointAngles_.at(j);
				} else {
					qInit_(j) = qInit[idx];
					idx++;
				}
			}
		} else {
			for (size_t j = 0; j != chain_->getNrOfJoints(); ++j) {
				qInit_(j) = sampleRandomAngle(lb(j), ub(j));
			}
		}

		int resultCode = ikSolver_->CartToJnt(qInit_, kdlPose, result);
		if (resultCode < 0) {
			// Unsuccessful
			continue;
		}

		VectorFloat resultingJointAngles(numActuatedJoints, 0.0);
		unsigned int idx = 0;
		for (size_t j = 0; j != chain_->getNrOfJoints(); ++j) {
			if (fixedJointAngles_.find(j) == fixedJointAngles_.end()) {
				resultingJointAngles[idx] = result(j);
				idx++;
			}
		}

		return std::make_pair(true, resultingJointAngles);
	}

	return std::make_pair(false, VectorFloat());
}

FloatType TracIKSolver::sampleRandomAngle(const FloatType &lower, const FloatType &upper) {
	std::uniform_real_distribution<> dis(lower, upper);
	return dis(*(randGen_));
}

std::unique_ptr<IKSolver> TracIKSolver::clone() const {
	std::unique_ptr<IKSolver> clonedIkSolver(new TracIKSolver(randGen_, urdfFile_, baseLink_, tipLink_));
	return std::move(clonedIkSolver);
}

void TracIKSolver::setFixedJoints(std::map<std::string, FloatType> &fixedJoints) {
	fixedJoints_ = fixedJoints;
}

bool TracIKSolver::inFixedJoints(const std::string &jointName) const {
	for (auto &entry : fixedJoints_) {
		if (entry.first.find(jointName) != std::string::npos) {
			return true;
		}
	}

	return false;
}

VectorFloat TracIKSolver::endEffectorVelocityFromJointVelocities(const VectorFloat &currentJointAngles,
        const VectorFloat &currentJointVelocities) {
	if (currentJointAngles.size() != chain_->getNrOfJoints() - fixedJointAngles_.size())
		ERROR("Current joint angles has wrong format");
	KDL::JntArray tauArr;
	tauArr.resize(chain_->getNrOfJoints());
	unsigned int idx = 0;
	for (size_t i = 0; i != chain_->getNrOfJoints(); ++i) {
		if (fixedJointAngles_.find(i) != fixedJointAngles_.end()) {
			q_(i) = fixedJointAngles_.at(i);
			tauArr(i) = 0.0;
		} else {
			q_(i) = currentJointAngles[idx];
			tauArr(i) = currentJointVelocities[idx];
			idx++;
		}
	}

	jacobianSolver_->JntToJac(q_, *(jacobian_.get()));
	KDL::Twist t;
	KDL::MultiplyJacobian(*(jacobian_.get()), tauArr, t);
	VectorFloat twist({t.vel.x(), t.vel.y(), t.vel.z(), t.rot.x(), t.rot.y(), t.rot.z()});
	return twist;
}

VectorFloat TracIKSolver::endEffectorForceFromTorque(const VectorFloat &currentJointAngles, const VectorFloat &tau) {
	if (currentJointAngles.size() != chain_->getNrOfJoints() - fixedJointAngles_.size()) {
		cout << "chain nr joints: " << chain_->getNrOfJoints() << endl;
		cout << "fixed joint angles size: " << fixedJointAngles_.size() << endl;
		ERROR("Current joint angles has wrong format");
	}
#ifdef EIGEN_GT_3_3_4
	if (tau.size() != chain_->getNrOfJoints())
		ERROR("Tau has wrong format");
	Vectordf tauEigen = toEigenVec(tau);
	KDL::JntArray q;
	q.resize(fixedJointsChain_->getNrOfJoints());
	for (size_t i = 0; i != chain_->getNrOfJoints(); ++i) {
		q(i) = currentJointAngles[i];
	}

	// Get the jacobian
	KDL::Jacobian jacobian(chain_->getNrOfJoints());
	jacobianSolver_->JntToJac(q, jacobian);
	Matrixdf jacobianTranspose = toEigenMatrix_(jacobian).transpose();
	VectorFloat eeForce = toStdVec<FloatType>(jacobianTranspose.completeOrthogonalDecomposition().pseudoInverse() * tauEigen);
	eeForce.resize(6);
	return eeForce;
#else
	WARNING("TrackIKSolver::endEffectorForceFromTorque requires Eigen 3.3.4 or greater");
	return VectorFloat();
#endif
}

void TracIKSolver::pinv( Eigen::MatrixXd & input_matrix) {
	Matrixdf pinv = Eigen::MatrixXd::Identity(input_matrix.rows(), input_matrix.cols());
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	pinv = svd.solve(pinv);
}

VectorFloat TracIKSolver::jointVelocitiesFromTwist(const VectorFloat &currentJointAngles, const VectorFloat &twist) {
	if (currentJointAngles.size() != chain_->getNrOfJoints() - fixedJointAngles_.size())
		ERROR("Current joint angles has wrong format");
	if (twist.size() != 6)
		ERROR("Twist vector must be of size 6");

	KDL::JntArray q;
	q.resize(fixedJointsChain_->getNrOfJoints());
	for (size_t i = 0; i != fixedJointsChain_->getNrOfJoints(); ++i) {
		q(i) = currentJointAngles[i];
	}

	KDL::Twist kdlTwist;
	kdlTwist.vel = KDL::Vector(twist[0], twist[1], twist[2]);
	kdlTwist.rot = KDL::Vector(twist[3], twist[4], twist[5]);
	KDL::JntArray qDot;
	qDot.resize(fixedJointsChain_->getNrOfJoints());
	int res = inverseJacobianSolver_->CartToJnt(q, kdlTwist, qDot);
	VectorFloat qDotVec(currentJointAngles.size(), 0.0);
	for (size_t i = 0; i != fixedJointsChain_->getNrOfJoints(); ++i) {
		qDotVec[i] = qDot(i);
	}

	return qDotVec;
}


VectorFloat TracIKSolver::inverseJacobian(const VectorFloat &q, const VectorFloat &x1, const VectorFloat &x2) const {
	VectorFloat deltaX = subtractVectors(x2, x1);
	for (size_t i = 0; i != q.size(); ++i) {
		q_(i) = q[i];
	}

	KDL::Twist twist;
	twist.vel = KDL::Vector(deltaX[0], deltaX[1], deltaX[2]);
	twist.rot = KDL::Vector(deltaX[3], deltaX[4], deltaX[5]);
	printVector(deltaX, "deltaX");
	KDL::JntArray qDot;
	qDot.resize(q.size());
	inverseJacobianSolver_->CartToJnt(q_, twist, qDot);

	VectorFloat qDotVec(q.size());
	for (size_t i = 0; i != qDotVec.size(); ++i) {
		qDotVec[i] = qDot(i);
	}

	return addVectors(q, qDotVec);
	/**auto qEigen = toEigenVec(q);
	auto x1Eigen = toEigenVec(x1);
	auto x2Eigen = toEigenVec(x2);

	auto deltaX = x2Eigen - x1Eigen;

	cout << "q: " << qEigen << endl;
	cout << "x1Eigen: " << x1Eigen << endl;
	cout << "x2Eigen: " << x2Eigen << endl;
	cout << "deltaX: " << deltaX << endl;

	for (size_t i = 0; i != q.size(); ++i) {
		q_(i) = q[i];
	}

	jacobianSolver_->JntToJac(q_, *(jacobian_.get()));

	// Put the jacobian into an eigen matrix
	Matrixdf jacobianMatrix = Matrixdf::Zero(6, chain_->getNrOfJoints());
	for (size_t i = 0; i != 6; ++i) {
		for (size_t j = 0; j != chain_->getNrOfJoints(); ++j) {
			jacobianMatrix(i, j) = (*(jacobian_.get()))(i, j);
		}
	}

	cout << "jacobianMatrix: " << endl << jacobianMatrix << endl;
	Matrixdf jTranspose = jacobianMatrix.transpose();

	Vectordf dotTheta = jTranspose * (jacobianMatrix * jTranspose).inverse() * deltaX;

	//Vectordf dotTheta = jacobianMatrix.colPivHouseholderQr().solve(deltaX);

	// Calculate the pseudo inverse of the jacobian matrix using orthogonal decomposition
	// and hope for the best
	//Matrixdf pInv = jacobianMatrix.completeOrthogonalDecomposition().pseudoInverse();
	//cout << "pInv: " << endl << pInv << endl;
	//auto dotTheta = pInv * deltaX;
	cout << "dotTheta: " << dotTheta << endl;
	Vectordf qNew = qEigen + dotTheta;

	cout << "qNew: " << qNew << endl;
	getchar();
	return toStdVec<FloatType>(qNew);*/
}

Matrixdf TracIKSolver::toEigenMatrix_(const KDL::Jacobian &jacobian) const {
	Matrixdf jacobianMatrix = Matrixdf::Zero(jacobian.rows(), jacobian.columns());
	for (size_t i = 0; i != jacobian.rows(); ++i) {
		for (size_t j = 0; j != jacobian.columns(); ++j) {
			jacobianMatrix(i, j) = jacobian(i, j);
		}
	}

	return jacobianMatrix;
}