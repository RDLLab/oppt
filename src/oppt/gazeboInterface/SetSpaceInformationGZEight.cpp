#include "GazeboInterface.hpp"

namespace oppt {

void GazeboInterface::makeJointVectorHelper_(const VectorString &jointNames,
        const std::unordered_map<std::string, VectorString> &redundancyMap,
        std::vector<JointPtr> &jointVector,
        std::vector<std::vector<JointPtr>> &redundantJoints) const {
	for (size_t i = 0; i != jointNames.size(); ++i) {
		JointPtr joint = worldJointMap_.at(world_->GetName()).at(jointNames[i]);
		if (!joint)
			ERROR("Joint '" + jointNames[i] + "' not in your model");
		jointVector.push_back(joint);
		std::vector<JointPtr> redJoints;
		if (redundancyMap.count(jointNames[i]) == 1) {
			VectorString redundantJointNames = redundancyMap.at(jointNames[i]);
			for (size_t j = 0; j != redundantJointNames.size(); ++j) {
				redJoints.push_back(worldJointMap_.at(world_->GetName()).at(redundantJointNames[j]));
			}
		}

		redundantJoints.push_back(redJoints);
	}
}

void GazeboInterface::makeLinkVectorHelper_(const VectorString &linkNames, std::vector<LinkPtr> &links) const {
	for (size_t i = 0; i != linkNames.size(); ++i) {		
		links.push_back(worldLinkMap_.at(world_->GetName()).at(linkNames[i]));
	}
}

void GazeboInterface::setStateSpaceInformation(const StateSpaceInformationPtr& stateSpaceInformation)
{
	stateSpaceInformation_ = stateSpaceInformation;
	VectorString jointPositionVec = stateSpaceInformation_->jointPositions;
	VectorString jointVelocityVec = stateSpaceInformation_->jointVelocities;
	VectorString linkPosesVec = stateSpaceInformation_->containedLinkPoses;
	VectorString linkPositionsXVec = stateSpaceInformation_->containedLinkPositionsX;	
	VectorString linkPositionsYVec = stateSpaceInformation_->containedLinkPositionsY;
	VectorString linkPositionsZVec = stateSpaceInformation_->containedLinkPositionsZ;
	VectorString linkOrientationsXVec = stateSpaceInformation_->containedLinkOrientationsX;
	VectorString linkOrientationsYVec = stateSpaceInformation_->containedLinkOrientationsY;
	VectorString linkOrientationsZVec = stateSpaceInformation_->containedLinkOrientationsZ;
	VectorString linkVelocitiesLinearVec = stateSpaceInformation_->containedLinkVelocitiesLinear;
	VectorString linkVelocitiesAngularVec = stateSpaceInformation_->containedLinkVelocitiesAngular;
	VectorString linkVelocitiesLinearXVec = stateSpaceInformation_->containedLinkLinearVelocitiesX;
	VectorString linkVelocitiesLinearYVec = stateSpaceInformation_->containedLinkLinearVelocitiesY;
	VectorString linkVelocitiesLinearZVec = stateSpaceInformation_->containedLinkLinearVelocitiesZ;
	VectorString linkVelocitiesAngularXVec = stateSpaceInformation_->containedLinkAngularVelocitiesX;
	VectorString linkVelocitiesAngularYVec = stateSpaceInformation_->containedLinkAngularVelocitiesY;
	VectorString linkVelocitiesAngularZVec = stateSpaceInformation_->containedLinkAngularVelocitiesZ;
	const SpaceVariables *orderedVariables = stateSpaceInformation_->orderedVariables.get();
	std::vector<VectorString> redundantJoints = stateSpaceInformation_->redundantJoints;
	unsigned int startIndex = 0;

	std::unordered_map<std::string, VectorString> redundancyMapPosition = generateRedundancyMap(jointPositionVec, redundantJoints);
	std::unordered_map<std::string, VectorString> redundancyMapVelocity = generateRedundancyMap(jointVelocityVec, redundantJoints);

	SetStateFunctions setPositionFunctions;
	SetStateFunctions setVelocityFunctions;

	// Make the vector of state joints
	std::vector<JointPtr> statePositionJoints;
	std::vector<std::vector<JointPtr>> redundantStatePositionJoints;
	makeJointVectorHelper_(jointPositionVec, redundancyMapPosition, statePositionJoints, redundantStatePositionJoints);

	std::vector<JointPtr> stateVelocityJoints;
	std::vector<std::vector<JointPtr>> redundantStateVelocityJoints;
	makeJointVectorHelper_(jointVelocityVec, redundancyMapVelocity, stateVelocityJoints, redundantStateVelocityJoints);

	std::vector<LinkPtr> stateLinkPoses;
	makeLinkVectorHelper_(linkPosesVec, stateLinkPoses);

	std::vector<LinkPtr> stateLinkPositionsX;
	makeLinkVectorHelper_(linkPositionsXVec, stateLinkPositionsX);	

	std::vector<LinkPtr> stateLinkPositionsY;
	makeLinkVectorHelper_(linkPositionsYVec, stateLinkPositionsY);

	std::vector<LinkPtr> stateLinkPositionsZ;
	makeLinkVectorHelper_(linkPositionsZVec, stateLinkPositionsZ);

	std::vector<LinkPtr> stateLinkOrientationsX;
	makeLinkVectorHelper_(linkOrientationsXVec, stateLinkOrientationsX);

	std::vector<LinkPtr> stateLinkOrientationsY;
	makeLinkVectorHelper_(linkOrientationsYVec, stateLinkOrientationsY);

	std::vector<LinkPtr> stateLinkOrientationsZ;
	makeLinkVectorHelper_(linkOrientationsZVec, stateLinkOrientationsZ);

	std::vector<LinkPtr> stateLinkVelocitiesLinear;
	makeLinkVectorHelper_(linkVelocitiesLinearVec, stateLinkVelocitiesLinear);

	std::vector<LinkPtr> stateLinkVelocitiesAngular;
	makeLinkVectorHelper_(linkVelocitiesAngularVec, stateLinkVelocitiesAngular);

	std::vector<LinkPtr> stateLinkVelocitiesLinearX;
	makeLinkVectorHelper_(linkVelocitiesLinearXVec, stateLinkVelocitiesLinearX);

	std::vector<LinkPtr> stateLinkVelocitiesLinearY;
	makeLinkVectorHelper_(linkVelocitiesLinearYVec, stateLinkVelocitiesLinearY);

	std::vector<LinkPtr> stateLinkVelocitiesLinearZ;
	makeLinkVectorHelper_(linkVelocitiesLinearZVec, stateLinkVelocitiesLinearZ);

	std::vector<LinkPtr> stateLinkVelocitiesAngularX;
	makeLinkVectorHelper_(linkVelocitiesAngularXVec, stateLinkVelocitiesAngularX);

	std::vector<LinkPtr> stateLinkVelocitiesAngularY;
	makeLinkVectorHelper_(linkVelocitiesAngularYVec, stateLinkVelocitiesAngularY);

	std::vector<LinkPtr> stateLinkVelocitiesAngularZ;
	makeLinkVectorHelper_(linkVelocitiesAngularZVec, stateLinkVelocitiesAngularZ);


	for (auto &spaceVariable : orderedVariables->spaceVariables) {
		switch (spaceVariable) {
		case SpaceVariable::JOINT_POSITIONS:
			if (jointPositionVec.size() > 0) {
				SetStateFunction setPosition = [this, statePositionJoints, redundantStatePositionJoints, startIndex](const VectorFloat & stateVec) {
					for (size_t i = 0; i != statePositionJoints.size(); ++i) {
						statePositionJoints[i]->SetPosition(0, stateVec[startIndex + i]);
						for (size_t j = 0; j != redundantStatePositionJoints[i].size(); ++j) {
							redundantStatePositionJoints[i][j]->SetPosition(i, stateVec[startIndex + i]);
						}

					}

				};

				GetStateFunction getPosition = [this, statePositionJoints, startIndex](VectorFloat & stateVec) {
					for (size_t i = 0; i != statePositionJoints.size(); i++) {
#ifdef GZ_GT_7
						stateVec[startIndex + i] = statePositionJoints[i]->Position(0);
#else
                                                stateVec[startIndex + i] = statePositionJoints[i]->GetAngle(0).Radian();
#endif						
					}
				};

				setPositionFunctions.push_back(setPosition);
				getStateFunctions_.push_back(getPosition);
				startIndex += jointPositionVec.size();
			}
			break;
		case SpaceVariable::JOINT_VELOCITIES:
			if (jointVelocityVec.size() > 0) {
				SetStateFunction setVelocity = [this, stateVelocityJoints, redundantStateVelocityJoints, startIndex](const VectorFloat & stateVec) {					
					for (size_t i = 0; i != stateVelocityJoints.size(); ++i) {
						setJointVelocityRecursive(stateVelocityJoints[i].get(), stateVec[startIndex + i]);
						for (size_t j = 0; j != redundantStateVelocityJoints[i].size(); ++j) {
							setJointVelocityRecursive(redundantStateVelocityJoints[i][j].get(), stateVec[startIndex + i]);
						}
					}
				};

				GetStateFunction getVelocity = [this, stateVelocityJoints, startIndex](VectorFloat & stateVec) {
					for (size_t i = 0; i != stateVelocityJoints.size(); ++i) {
						stateVec[startIndex + i] = stateVelocityJoints[i]->GetVelocity(0);
					}
				};

				setVelocityFunctions.push_back(setVelocity);
				getStateFunctions_.push_back(getVelocity);
				startIndex += jointVelocityVec.size();
			}
			break;
		case SpaceVariable::LINK_POSES:
			if (linkPosesVec.size() > 0) {
				SetStateFunction setLinkPoses = [this, stateLinkPoses, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkPoses.size(); ++i) {
						GZPose linkPose(stateVec[idx],
						                stateVec[idx + 1],
						                stateVec[idx + 2],
						                stateVec[idx + 3],
						                stateVec[idx + 4],
						                stateVec[idx + 5]);
						stateLinkPoses[i]->SetWorldPose(linkPose);
						idx += 6;
					}
				};

				GetStateFunction getLinkPoses = [this, stateLinkPoses, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;					
					for (size_t i = 0; i != stateLinkPoses.size(); ++i) {
#ifdef GZ_GT_7
						pose = stateLinkPoses[i]->WorldPose();
						GZVector3 orientation = pose.Rot().Euler();
						stateVec[idx] = pose.Pos().X();
						stateVec[idx + 1] = pose.Pos().Y();
						stateVec[idx + 2] = pose.Pos().Z();
						stateVec[idx + 3] = orientation.X();
						stateVec[idx + 4] = orientation.Y();
						stateVec[idx + 5] = orientation.Z();
#else
                                                pose = stateLinkPoses[i]->GetWorldPose();
						GZVector3 orientation = pose.rot.GetAsEuler();
						stateVec[idx] = pose.pos.x;
						stateVec[idx + 1] = pose.pos.y;
						stateVec[idx + 2] = pose.pos.z;
						stateVec[idx + 3] = orientation.x;
						stateVec[idx + 4] = orientation.y;
						stateVec[idx + 5] = orientation.z;
#endif
						idx += 6;
					}					
				};

				setPositionFunctions.push_back(setLinkPoses);
				getStateFunctions_.push_back(getLinkPoses);
				startIndex += linkPosesVec.size() * 6;
			}
			break;
		case SpaceVariable::LINK_POSITIONS_X:
			if (linkPositionsXVec.size() > 0) {
				SetStateFunction setLinkPositionsX = [this, stateLinkPositionsX, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkPositionsX.size(); ++i) {
#ifdef GZ_GT_7
						auto currentPose = stateLinkPositionsX[i]->WorldPose();
						GZVector3 newPosition(stateVec[idx], currentPose.Pos().Y(), currentPose.Pos().Z());
						currentPose.Set(newPosition, currentPose.Rot());
#else
                                                auto currentPose = stateLinkPositionsX[i]->GetWorldPose();
						GZVector3 newPosition(stateVec[idx], currentPose.pos.y, currentPose.pos.z);
						currentPose.Set(newPosition, currentPose.rot);
#endif
						stateLinkPositionsX[i]->SetWorldPose(currentPose);						
						idx++;
					}
				};

				GetStateFunction getLinkPositionsX = [this, stateLinkPositionsX, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					for (size_t i = 0; i != stateLinkPositionsX.size(); ++i) {
#ifdef GZ_GT_7
						pose = stateLinkPositionsX[i]->WorldPose();
						stateVec[idx] = pose.Pos().X();
#else
                                                pose = stateLinkPositionsX[i]->GetWorldPose();
						stateVec[idx] = pose.pos.x;
#endif
						idx += 1;
					}
				};

				setPositionFunctions.push_back(setLinkPositionsX);
				getStateFunctions_.push_back(getLinkPositionsX);
				startIndex += linkPositionsXVec.size();
			}
			break;
		case SpaceVariable::LINK_POSITIONS_Y:
			if (linkPositionsYVec.size() > 0) {
				SetStateFunction setLinkPositionsY = [this, stateLinkPositionsY, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkPositionsY.size(); ++i) {
#ifdef GZ_GT_7
						auto currentPose = stateLinkPositionsY[i]->WorldPose();
						GZVector3 newPosition(currentPose.Pos().X(), stateVec[idx], currentPose.Pos().Z());
						currentPose.Set(newPosition, currentPose.Rot());
#else
                                                auto currentPose = stateLinkPositionsY[i]->GetWorldPose();
						GZVector3 newPosition(currentPose.pos.x, stateVec[idx], currentPose.pos.z);
						currentPose.Set(newPosition, currentPose.rot);
#endif
						stateLinkPositionsY[i]->SetWorldPose(currentPose);
						idx++;
					}
				};

				GetStateFunction getLinkPositionsY = [this, stateLinkPositionsY, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					for (size_t i = 0; i != stateLinkPositionsY.size(); ++i) {
#ifdef GZ_GT_7
						pose = stateLinkPositionsY[i]->WorldPose();
						stateVec[idx] = pose.Pos().Y();
#else
                                                pose = stateLinkPositionsY[i]->GetWorldPose();
						stateVec[idx] = pose.pos.y;
#endif
						idx += 1;
					}
				};

				setPositionFunctions.push_back(setLinkPositionsY);
				getStateFunctions_.push_back(getLinkPositionsY);
				startIndex += linkPositionsYVec.size();
			}
			break;
		case SpaceVariable::LINK_POSITIONS_Z:
			if (linkPositionsZVec.size() > 0) {
				SetStateFunction setLinkPositionsZ = [this, stateLinkPositionsZ, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkPositionsZ.size(); ++i) {
#ifdef GZ_GT_7
						auto currentPose = stateLinkPositionsZ[i]->WorldPose();
						GZVector3 newPosition(currentPose.Pos().X(), currentPose.Pos().Y(), stateVec[idx]);
						currentPose.Set(newPosition, currentPose.Rot());
#else
                                                auto currentPose = stateLinkPositionsZ[i]->GetWorldPose();
						GZVector3 newPosition(currentPose.pos.x, currentPose.pos.y, stateVec[idx]);
						currentPose.Set(newPosition, currentPose.rot);
#endif
						stateLinkPositionsZ[i]->SetWorldPose(currentPose);
						idx++;
					}
				};

				GetStateFunction getLinkPositionsZ = [this, stateLinkPositionsZ, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					for (size_t i = 0; i != stateLinkPositionsZ.size(); ++i) {
#ifdef GZ_GT_7
						pose = stateLinkPositionsZ[i]->WorldPose();
						stateVec[idx] = pose.Pos().Z();
#else
                                                pose = stateLinkPositionsZ[i]->GetWorldPose();
						stateVec[idx] = pose.pos.z;
#endif
						idx += 1;
					}
				};

				setPositionFunctions.push_back(setLinkPositionsZ);
				getStateFunctions_.push_back(getLinkPositionsZ);
				startIndex += linkPositionsZVec.size();
			}
			break;
		case SpaceVariable::LINK_ORIENTATIONS_X:
			if (linkOrientationsXVec.size() > 0) {
				SetStateFunction setLinkOrientationsX = [this, stateLinkOrientationsX, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkOrientationsX.size(); ++i) {
#ifdef GZ_GT_7
						auto currentPose = stateLinkOrientationsX[i]->WorldPose();
						auto currentEuler = currentPose.Rot().Euler();
#else
                                                auto currentPose = stateLinkOrientationsX[i]->GetWorldPose();
						auto currentEuler = currentPose.rot.GetAsEuler();
#endif
						GZVector3 newRot(stateVec[idx], currentEuler[1], currentEuler[2]);
#ifdef GZ_GT_7
						currentPose.Set(currentPose.Pos(), newRot);
#else
                                                currentPose.Set(currentPose.pos, newRot);
#endif
						stateLinkOrientationsX[i]->SetWorldPose(currentPose);
						idx += 1;
					}
				};

				GetStateFunction getLinkOrientationsX = [this, stateLinkOrientationsX, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					for (size_t i = 0; i != stateLinkOrientationsX.size(); ++i) {
#ifdef GZ_GT_7
						pose = stateLinkOrientationsX[i]->WorldPose();
						stateVec[idx] = pose.Rot().Euler().X();
#else
                                                pose = stateLinkOrientationsX[i]->GetWorldPose();
						stateVec[idx] = pose.rot.GetAsEuler().x;
#endif
						idx += 1;
					}

				};

				setPositionFunctions.push_back(setLinkOrientationsX);
				getStateFunctions_.push_back(getLinkOrientationsX);
				startIndex += linkOrientationsXVec.size();
			}
			break;
		case SpaceVariable::LINK_ORIENTATIONS_Y:
			if (linkOrientationsYVec.size() > 0) {
				SetStateFunction setLinkOrientationsY = [this, stateLinkOrientationsY, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkOrientationsY.size(); ++i) {
#ifdef GZ_GT_7
						auto currentPose = stateLinkOrientationsY[i]->WorldPose();
						auto currentEuler = currentPose.Rot().Euler();
						GZVector3 newRot(currentEuler[0], stateVec[idx], currentEuler[2]);
						currentPose.Set(currentPose.Pos(), newRot);
#else
                                                auto currentPose = stateLinkOrientationsY[i]->GetWorldPose();
						auto currentEuler = currentPose.rot.GetAsEuler();
						GZVector3 newRot(currentEuler[0], stateVec[idx], currentEuler[2]);
						currentPose.Set(currentPose.pos, newRot);
#endif
						stateLinkOrientationsY[i]->SetWorldPose(currentPose);
						idx += 1;
					}
				};

				GetStateFunction getLinkOrientationsY = [this, stateLinkOrientationsY, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					for (size_t i = 0; i != stateLinkOrientationsY.size(); ++i) {
#ifdef GZ_GT_7
						pose = stateLinkOrientationsY[i]->WorldPose();
						stateVec[idx] = pose.Rot().Euler().Y();
#else
                                                pose = stateLinkOrientationsY[i]->GetWorldPose();
						stateVec[idx] = pose.rot.GetAsEuler().y;
#endif
						idx += 1;
					}

				};

				setPositionFunctions.push_back(setLinkOrientationsY);
				getStateFunctions_.push_back(getLinkOrientationsY);
				startIndex += linkOrientationsYVec.size();
			}
			break;
		case SpaceVariable::LINK_ORIENTATIONS_Z:
			if (linkOrientationsZVec.size() > 0) {
				SetStateFunction setLinkOrientationsZ = [this, stateLinkOrientationsZ, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkOrientationsZ.size(); ++i) {
#ifdef GZ_GT_7
						auto currentPose = stateLinkOrientationsZ[i]->WorldPose();
						auto currentEuler = currentPose.Rot().Euler();
						GZVector3 newRot(currentEuler[0], currentEuler[1], stateVec[idx]);
						currentPose.Set(currentPose.Pos(), newRot);
#else
                                                auto currentPose = stateLinkOrientationsZ[i]->GetWorldPose();
						auto currentEuler = currentPose.rot.GetAsEuler();
						GZVector3 newRot(currentEuler[0], currentEuler[1], stateVec[idx]);
						currentPose.Set(currentPose.pos, newRot);
#endif
						stateLinkOrientationsZ[i]->SetWorldPose(currentPose);
						idx += 1;
					}
				};

				GetStateFunction getLinkOrientationsZ = [this, stateLinkOrientationsZ, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					for (size_t i = 0; i != stateLinkOrientationsZ.size(); ++i) {
#ifdef GZ_GT_7
						pose = stateLinkOrientationsZ[i]->WorldPose();
						stateVec[idx] = pose.Rot().Euler().Z();
#else
                                                pose = stateLinkOrientationsZ[i]->GetWorldPose();
						stateVec[idx] = pose.rot.GetAsEuler().z;
#endif
						idx += 1;
					}

				};

				setPositionFunctions.push_back(setLinkOrientationsZ);
				getStateFunctions_.push_back(getLinkOrientationsZ);
				startIndex += linkOrientationsZVec.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR:
			if (linkVelocitiesLinearVec.size() > 0) {
				SetStateFunction setLinearVelocities = [this, stateLinkVelocitiesLinear, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesLinear.size(); ++i) {
						GZVector3 vel(stateVec[idx], stateVec[idx + 1], stateVec[idx + 2]);
						stateLinkVelocitiesLinear[i]->SetLinearVel(vel);
						idx += 3;
					}

				};

				GetStateFunction getLinearVelocities = [this, stateLinkVelocitiesLinear, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesLinear.size(); ++i) {
#ifdef GZ_GT_7
						vel = stateLinkVelocitiesLinear[i]->WorldLinearVel();
						stateVec[idx] = vel.X();
						stateVec[idx + 1] = vel.Y();
						stateVec[idx + 2] = vel.Z();
#else
                                                vel = stateLinkVelocitiesLinear[i]->GetWorldLinearVel();
						stateVec[idx] = vel.x;
						stateVec[idx + 1] = vel.y;
						stateVec[idx + 2] = vel.z;
#endif
						idx += 3;
					}
				};

				setVelocityFunctions.push_back(setLinearVelocities);
				getStateFunctions_.push_back(getLinearVelocities);
				startIndex += linkVelocitiesLinearVec.size() * 3;
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR:
			if (linkVelocitiesAngularVec.size() > 0) {
				SetStateFunction setAngularVelocities = [this, stateLinkVelocitiesAngular, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesAngular.size(); ++i) {
						GZVector3 vel(stateVec[idx], stateVec[idx + 1], stateVec[idx + 2]);
						stateLinkVelocitiesAngular[i]->SetAngularVel(vel);
						idx += 3;
					}
				};

				GetStateFunction getAngularVelocities = [this, stateLinkVelocitiesAngular, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesAngular.size(); ++i) {
#ifdef GZ_GT_7
						vel = stateLinkVelocitiesAngular[i]->WorldAngularVel();
						stateVec[idx] = vel.X();
						stateVec[idx + 1] = vel.Y();
						stateVec[idx + 2] = vel.Z();
#else
                                                vel = stateLinkVelocitiesAngular[i]->GetWorldAngularVel();
						stateVec[idx] = vel.x;
						stateVec[idx + 1] = vel.y;
						stateVec[idx + 2] = vel.z;
#endif
						idx += 3;
					}
				};

				setVelocityFunctions.push_back(setAngularVelocities);
				getStateFunctions_.push_back(getAngularVelocities);
				startIndex += linkVelocitiesAngularVec.size() * 3;
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR_X:
			if (linkVelocitiesLinearXVec.size() > 0) {
				SetStateFunction setLinearVelocitiesX = [this, stateLinkVelocitiesLinearX, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesLinearX.size(); ++i) {
#ifdef GZ_GT_7
						GZVector3 currentVel = stateLinkVelocitiesLinearX[i]->WorldLinearVel();
						GZVector3 vel(stateVec[idx], currentVel.Y(), currentVel.Z());
#else
                                                GZVector3 currentVel = stateLinkVelocitiesLinearX[i]->GetWorldLinearVel();
						GZVector3 vel(stateVec[idx], currentVel.y, currentVel.z);
#endif
						stateLinkVelocitiesLinearX[i]->SetLinearVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getLinearVelocitiesX = [this, stateLinkVelocitiesLinearX, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesLinearX.size(); ++i) {
#ifdef GZ_GT_7
						stateVec[idx] = stateLinkVelocitiesLinearX[i]->WorldLinearVel().X();
#else
                                                stateVec[idx] = stateLinkVelocitiesLinearX[i]->GetWorldLinearVel().x;
#endif
						idx += 1;
					}
				};

				setVelocityFunctions.push_back(setLinearVelocitiesX);
				getStateFunctions_.push_back(getLinearVelocitiesX);
				startIndex += linkVelocitiesLinearXVec.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR_Y:
			if (linkVelocitiesLinearYVec.size() > 0) {
				SetStateFunction setLinearVelocitiesY = [this, stateLinkVelocitiesLinearY, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesLinearY.size(); ++i) {
#ifdef GZ_GT_7
						GZVector3 currentVel = stateLinkVelocitiesLinearY[i]->WorldLinearVel();
						GZVector3 vel(currentVel.X(), stateVec[idx], currentVel.Z());
#else
                                                GZVector3 currentVel = stateLinkVelocitiesLinearY[i]->GetWorldLinearVel();
						GZVector3 vel(currentVel.x, stateVec[idx], currentVel.z);
#endif
						stateLinkVelocitiesLinearY[i]->SetLinearVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getLinearVelocitiesY = [this, stateLinkVelocitiesLinearY, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesLinearY.size(); ++i) {
#ifdef GZ_GT_7
						stateVec[idx] = stateLinkVelocitiesLinearY[i]->WorldLinearVel().Y();
#else
                                                stateVec[idx] = stateLinkVelocitiesLinearY[i]->GetWorldLinearVel().y;
#endif
						idx += 1;
					}
				};

				setVelocityFunctions.push_back(setLinearVelocitiesY);
				getStateFunctions_.push_back(getLinearVelocitiesY);
				startIndex += linkVelocitiesLinearYVec.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR_Z:
			if (linkVelocitiesLinearZVec.size() > 0) {
				SetStateFunction setLinearVelocitiesZ = [this, stateLinkVelocitiesLinearZ, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesLinearZ.size(); ++i) {
#ifdef GZ_GT_7
						GZVector3 currentVel = stateLinkVelocitiesLinearZ[i]->WorldLinearVel();
						GZVector3 vel(currentVel.X(), currentVel.Y(), stateVec[idx]);
#else
                                                GZVector3 currentVel = stateLinkVelocitiesLinearZ[i]->GetWorldLinearVel();
						GZVector3 vel(currentVel.x, currentVel.y, stateVec[idx]);
#endif
						stateLinkVelocitiesLinearZ[i]->SetLinearVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getLinearVelocitiesZ = [this, stateLinkVelocitiesLinearZ, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesLinearZ.size(); ++i) {
#ifdef GZ_GT_7
						stateVec[idx] = stateLinkVelocitiesLinearZ[i]->WorldLinearVel().Z();
#else
                                                stateVec[idx] = stateLinkVelocitiesLinearZ[i]->GetWorldLinearVel().z;
#endif
						idx += 1;
					}
				};

				setVelocityFunctions.push_back(setLinearVelocitiesZ);
				getStateFunctions_.push_back(getLinearVelocitiesZ);
				startIndex += linkVelocitiesLinearZVec.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR_X:
			if (linkVelocitiesAngularXVec.size() > 0) {
				SetStateFunction setAngularVelocitiesX = [this, stateLinkVelocitiesAngularX, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesAngularX.size(); ++i) {
#ifdef GZ_GT_7
						GZVector3 currentVel = stateLinkVelocitiesAngularX[i]->WorldAngularVel();
						GZVector3 vel(stateVec[idx], currentVel.Y(), currentVel.Z());
#else
                                                GZVector3 currentVel = stateLinkVelocitiesAngularX[i]->GetWorldAngularVel();
						GZVector3 vel(stateVec[idx], currentVel.y, currentVel.z);
#endif
						stateLinkVelocitiesAngularX[i]->SetAngularVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getAngularVelocitiesX = [this, stateLinkVelocitiesAngularX, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesAngularX.size(); ++i) {
#ifdef GZ_GT_7
						vel = stateLinkVelocitiesAngularX[i]->WorldAngularVel();
						stateVec[idx] = vel.X();
#else
                                                vel = stateLinkVelocitiesAngularX[i]->GetWorldAngularVel();
						stateVec[idx] = vel.x;
#endif
						idx += 1;
					}
				};

				setVelocityFunctions.push_back(setAngularVelocitiesX);
				getStateFunctions_.push_back(getAngularVelocitiesX);
				startIndex += linkVelocitiesAngularXVec.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR_Y:
			if (linkVelocitiesAngularYVec.size() > 0) {
				SetStateFunction setAngularVelocitiesY = [this, stateLinkVelocitiesAngularY, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesAngularY.size(); ++i) {
#ifdef GZ_GT_7
						GZVector3 currentVel = stateLinkVelocitiesAngularY[i]->WorldAngularVel();
						GZVector3 vel(currentVel.X(), stateVec[idx], currentVel.Z());
#else
                                                GZVector3 currentVel = stateLinkVelocitiesAngularY[i]->GetWorldAngularVel();
						GZVector3 vel(currentVel.x, stateVec[idx], currentVel.z);
#endif
						stateLinkVelocitiesAngularY[i]->SetAngularVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getAngularVelocitiesY = [this, stateLinkVelocitiesAngularY, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesAngularY.size(); ++i) {
#ifdef GZ_GT_7
						vel = stateLinkVelocitiesAngularY[i]->WorldAngularVel();
						stateVec[idx] = vel.Y();
#else
                                                vel = stateLinkVelocitiesAngularY[i]->GetWorldAngularVel();
						stateVec[idx] = vel.y;
#endif
						idx += 1;
					}
				};

				setVelocityFunctions.push_back(setAngularVelocitiesY);
				getStateFunctions_.push_back(getAngularVelocitiesY);
				startIndex += linkVelocitiesAngularYVec.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR_Z:
			if (linkVelocitiesAngularZVec.size() > 0) {
				SetStateFunction setAngularVelocitiesZ = [this, stateLinkVelocitiesAngularZ, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != stateLinkVelocitiesAngularZ.size(); ++i) {
#ifdef GZ_GT_7
						GZVector3 currentVel = stateLinkVelocitiesAngularZ[i]->WorldAngularVel();
						GZVector3 vel(currentVel.X(), currentVel.Y(), stateVec[idx]);
#else
                                                GZVector3 currentVel = stateLinkVelocitiesAngularZ[i]->GetWorldAngularVel();
						GZVector3 vel(currentVel.x, currentVel.y, stateVec[idx]);
#endif
						stateLinkVelocitiesAngularZ[i]->SetAngularVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getAngularVelocitiesZ = [this, stateLinkVelocitiesAngularZ, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != stateLinkVelocitiesAngularZ.size(); ++i) {
#ifdef GZ_GT_7
						vel = stateLinkVelocitiesAngularZ[i]->WorldAngularVel();
						stateVec[idx] = vel.Z();
#else
                                                vel = stateLinkVelocitiesAngularZ[i]->GetWorldAngularVel();
						stateVec[idx] = vel.z;
#endif
						idx += 1;
					}
				};

				setVelocityFunctions.push_back(setAngularVelocitiesZ);
				getStateFunctions_.push_back(getAngularVelocitiesZ);
				startIndex += linkVelocitiesAngularZVec.size();
			}
			break;
		default:
			ERROR("Unknown space variable");
			break;
		}
	}

	// Position needs to be set first, followed by velocity

	for (size_t i = 0; i != setPositionFunctions.size(); ++i) {
		setStateFunctions_.push_back(setPositionFunctions[i]);
	}
	for (size_t i = 0; i != setVelocityFunctions.size(); ++i) {
		setStateFunctions_.push_back(setVelocityFunctions[i]);
	}

	stateSpaceDimension_ = startIndex;
}

void GazeboInterface::setObservationSpaceInformation(const ObservationSpaceInformationPtr & observationSpaceInformation)
{
	observationSpaceInformation_ = observationSpaceInformation;
	VectorString observedJointPositions = observationSpaceInformation_->jointPositions;
	VectorString observedJointVelocities = observationSpaceInformation_->jointVelocities;
	VectorString observedLinkPoses = observationSpaceInformation_->containedLinkPoses;
	VectorString observedLinkPositionsX = observationSpaceInformation_->containedLinkPositionsX;
	VectorString observedLinkPositionsY = observationSpaceInformation_->containedLinkPositionsY;
	VectorString observedLinkPositionsZ = observationSpaceInformation_->containedLinkPositionsZ;
	VectorString observedLinkVelocitiesLinear = observationSpaceInformation_->containedLinkVelocitiesLinear;
	VectorString observedLinkVelocitiesAngular = observationSpaceInformation_->containedLinkVelocitiesAngular;
	VectorString observedLinkVelocitiesLinearX = observationSpaceInformation->containedLinkLinearVelocitiesX;
	VectorString observedLinkVelocitiesLinearY = observationSpaceInformation->containedLinkLinearVelocitiesY;
	VectorString observedLinkVelocitiesLinearZ = observationSpaceInformation->containedLinkLinearVelocitiesZ;
	VectorString observedLinkVelocitiesAngularX = observationSpaceInformation->containedLinkAngularVelocitiesX;
	VectorString observedLinkVelocitiesAngularY = observationSpaceInformation->containedLinkAngularVelocitiesY;
	VectorString observedLinkVelocitiesAngularZ = observationSpaceInformation->containedLinkAngularVelocitiesZ;
	const SpaceVariables *orderedVariables = observationSpaceInformation_->orderedVariables.get();
	unsigned int startIndex = 0;

	std::vector<JointPtr> observationPositionJoints;
	std::vector<std::vector<JointPtr>> redundantObservationPositionJoints;
	makeJointVectorHelper_(observedJointPositions,
	                       std::unordered_map<std::string, VectorString>(),
	                       observationPositionJoints,
	                       redundantObservationPositionJoints);

	std::vector<JointPtr> observationVelocityJoints;
	makeJointVectorHelper_(observedJointVelocities,
	                       std::unordered_map<std::string, VectorString>(),
	                       observationVelocityJoints,
	                       redundantObservationPositionJoints);

	std::vector<LinkPtr> observationLinkPoses;
	makeLinkVectorHelper_(observedLinkPoses, observationLinkPoses);

	std::vector<LinkPtr> observationLinkPositionsX;
	makeLinkVectorHelper_(observedLinkPositionsX, observationLinkPositionsX);

	std::vector<LinkPtr> observationLinkPositionsY;
	makeLinkVectorHelper_(observedLinkPositionsY, observationLinkPositionsY);

	std::vector<LinkPtr> observationLinkPositionsZ;
	makeLinkVectorHelper_(observedLinkPositionsZ, observationLinkPositionsZ);

	std::vector<LinkPtr> observationLinkVelocitiesLinear;
	makeLinkVectorHelper_(observedLinkVelocitiesLinear, observationLinkVelocitiesLinear);

	std::vector<LinkPtr> observationLinkVelocitiesAngular;
	makeLinkVectorHelper_(observedLinkVelocitiesAngular, observationLinkVelocitiesAngular);

	std::vector<LinkPtr> observationLinkVelocitiesLinearX;
	makeLinkVectorHelper_(observedLinkVelocitiesLinearX, observationLinkVelocitiesLinearX);

	std::vector<LinkPtr> observationLinkVelocitiesLinearY;
	makeLinkVectorHelper_(observedLinkVelocitiesLinearY, observationLinkVelocitiesLinearY);

	std::vector<LinkPtr> observationLinkVelocitiesLinearZ;
	makeLinkVectorHelper_(observedLinkVelocitiesLinearZ, observationLinkVelocitiesLinearZ);

	std::vector<LinkPtr> observationLinkVelocitiesAngularX;
	makeLinkVectorHelper_(observedLinkVelocitiesAngularX, observationLinkVelocitiesAngularX);

	std::vector<LinkPtr> observationLinkVelocitiesAngularY;
	makeLinkVectorHelper_(observedLinkVelocitiesAngularY, observationLinkVelocitiesAngularY);

	std::vector<LinkPtr> observationLinkVelocitiesAngularZ;
	makeLinkVectorHelper_(observedLinkVelocitiesAngularZ, observationLinkVelocitiesAngularZ);


	for (auto &spaceVariable : orderedVariables->spaceVariables) {
		switch (spaceVariable) {
		case SpaceVariable::JOINT_POSITIONS:
			if (observedJointPositions.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationPositionJoints,
				startIndex](VectorFloat & observationVec) {
					for (size_t i = 0; i != observationPositionJoints.size(); i++) {
#ifdef GZ_GT_7
						observationVec[startIndex + i] = observationPositionJoints[i]->Position(0);
#else
                                                observationVec[startIndex + i] = observationPositionJoints[i]->GetAngle(0).Radian();
#endif
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedJointPositions.size();
			}
			break;
		case SpaceVariable::JOINT_VELOCITIES:
			if (observedJointVelocities.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationVelocityJoints,
				startIndex](VectorFloat & observationVec) {
					JointMap jm = worldJointMap_[world_->GetName()];
					for (size_t i = 0; i != observationVelocityJoints.size(); i++) {
						observationVec[startIndex + i] = observationVelocityJoints[i]->GetVelocity(0);
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedJointVelocities.size();
			}
			break;
		case SpaceVariable::LINK_POSES:
			if (observedLinkPoses.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkPoses,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZPose linkPose;
					GZVector3 rotation;
					for (size_t i = 0; i != observationLinkPoses.size(); ++i) {
#ifdef GZ_GT_7
						linkPose = observationLinkPoses[i]->WorldPose();
						rotation = linkPose.Rot().Euler();
						observationVec[idx] = linkPose.Pos().X();
						observationVec[idx + 1] = linkPose.Pos().Y();
						observationVec[idx + 2] = linkPose.Pos().Z();
						observationVec[idx + 3] = rotation.X();
						observationVec[idx + 4] = rotation.Y();
						observationVec[idx + 5] = rotation.Z();
#else
                                                linkPose = observationLinkPoses[i]->GetWorldPose();
						rotation = linkPose.rot.GetAsEuler();
						observationVec[idx] = linkPose.pos.x;
						observationVec[idx + 1] = linkPose.pos.y;
						observationVec[idx + 2] = linkPose.pos.z;
						observationVec[idx + 3] = rotation.x;
						observationVec[idx + 4] = rotation.y;
						observationVec[idx + 5] = rotation.z;
#endif
						idx += 6;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkPoses.size() * 6;
			}
			break;
		case SpaceVariable::LINK_POSITIONS_X:
			if (observedLinkPositionsX.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkPositionsX,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != observationLinkPositionsX.size(); ++i) {
#ifdef GZ_GT_7
						observationVec[idx] = observationLinkPositionsX[i]->WorldPose().Pos().X();
#else
                                                observationVec[idx] = observationLinkPositionsX[i]->GetWorldPose().pos.x;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkPositionsX.size();
			}
			break;
		case SpaceVariable::LINK_POSITIONS_Y:
			if (observedLinkPositionsY.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkPositionsY,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != observationLinkPositionsY.size(); ++i) {
#ifdef GZ_GT_7
						observationVec[idx] = observationLinkPositionsY[i]->WorldPose().Pos().X();
#else
                                                observationVec[idx] = observationLinkPositionsY[i]->GetWorldPose().pos.x;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkPositionsY.size();
			}
			break;
		case SpaceVariable::LINK_POSITIONS_Z:
			if (observedLinkPositionsZ.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkPositionsZ,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					for (size_t i = 0; i != observationLinkPositionsZ.size(); ++i) {
#ifdef GZ_GT_7
						observationVec[idx] = observationLinkPositionsZ[i]->WorldPose().Pos().X();
#else
                                                observationVec[idx] = observationLinkPositionsZ[i]->GetWorldPose().pos.x;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkPositionsZ.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR:
			if (observedLinkVelocitiesLinear.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesLinear,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;					
					for (size_t i = 0; i != observationLinkVelocitiesLinear.size(); ++i) {
#ifdef GZ_GT_7
						vel = observationLinkVelocitiesLinear[i]->WorldLinearVel();
						observationVec[idx] = vel.X();
						observationVec[idx + 1] = vel.Y();
						observationVec[idx + 2] = vel.Z();
#else
                                                vel = observationLinkVelocitiesLinear[i]->GetWorldLinearVel();
						observationVec[idx] = vel.x;
						observationVec[idx + 1] = vel.y;
						observationVec[idx + 2] = vel.z;
#endif
						idx += 3;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesLinear.size() * 3;
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR:
			if (observedLinkVelocitiesAngular.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesAngular,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;					
					for (size_t i = 0; i != observationLinkVelocitiesAngular.size(); ++i) {
#ifdef GZ_GT_7
						vel = observationLinkVelocitiesAngular[i]->WorldAngularVel();
						observationVec[idx] = vel.X();
						observationVec[idx + 1] = vel.Y();
						observationVec[idx + 2] = vel.Z();
#else
                                                vel = observationLinkVelocitiesAngular[i]->GetWorldAngularVel();
						observationVec[idx] = vel.x;
						observationVec[idx + 1] = vel.y;
						observationVec[idx + 2] = vel.z;
#endif
						idx += 3;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesAngular.size() * 3;
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR_X:
			if (observedLinkVelocitiesLinearX.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesLinearX,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;					
					for (size_t i = 0; i != observationLinkVelocitiesLinearX.size(); ++i) {
#ifdef GZ_GT_7
						observationVec[idx] = observationLinkVelocitiesLinearX[i]->WorldLinearVel().X();
#else
                                                observationVec[idx] = observationLinkVelocitiesLinearX[i]->GetWorldLinearVel().x;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesLinearX.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR_Y:
			if (observedLinkVelocitiesLinearY.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesLinearY,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;					
					for (size_t i = 0; i != observationLinkVelocitiesLinearY.size(); ++i) {
#ifdef GZ_GT_7
						observationVec[idx] = observationLinkVelocitiesLinearY[i]->WorldLinearVel().Y();
#else
                                                observationVec[idx] = observationLinkVelocitiesLinearY[i]->GetWorldLinearVel().y;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesLinearY.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_LINEAR_Z:
			if (observedLinkVelocitiesLinearZ.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesLinearZ,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;					
					for (size_t i = 0; i != observationLinkVelocitiesLinearZ.size(); ++i) {	
#ifdef GZ_GT_7					
						observationVec[idx] = observationLinkVelocitiesLinearZ[i]->WorldLinearVel().Z();
#else
                                                observationVec[idx] = observationLinkVelocitiesLinearZ[i]->GetWorldLinearVel().z;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesLinearZ.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR_X:
			if (observedLinkVelocitiesAngularX.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesAngularX,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != observationLinkVelocitiesAngularX.size(); ++i) {	
#ifdef GZ_GT_7					
						observationVec[idx] = observationLinkVelocitiesAngularX[i]->WorldAngularVel().X();
#else
                                                observationVec[idx] = observationLinkVelocitiesAngularX[i]->GetWorldAngularVel().x;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesAngularX.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR_Y:
			if (observedLinkVelocitiesAngularY.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesAngularY,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != observationLinkVelocitiesAngularY.size(); ++i) {
#ifdef GZ_GT_7						
						observationVec[idx] = observationLinkVelocitiesAngularY[i]->WorldAngularVel().Y();
#else
                                                observationVec[idx] = observationLinkVelocitiesAngularY[i]->GetWorldAngularVel().y;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesAngularY.size();
			}
			break;
		case SpaceVariable::LINK_VELOCITIES_ANGULAR_Z:
			if (observedLinkVelocitiesAngularZ.size() > 0) {
				GetObservationFunction obsFunct = [this,
				                                   observationLinkVelocitiesAngularZ,
				startIndex](VectorFloat & observationVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					for (size_t i = 0; i != observationLinkVelocitiesAngularZ.size(); ++i) {	
#ifdef GZ_GT_7					
						observationVec[idx] = observationLinkVelocitiesAngularZ[i]->WorldAngularVel().Z();
#else
                                                observationVec[idx] = observationLinkVelocitiesAngularZ[i]->GetWorldAngularVel().z;
#endif
						idx += 1;
					}
				};

				getObservationFunctions_.push_back(obsFunct);
				startIndex += observedLinkVelocitiesAngularZ.size();
			}
			break;
		default:
			ERROR("Observation variable not recognized");
			break;
		}
	}

	GetObservationFunction sensorObservations = [this, startIndex](VectorFloat & observationVec) {
		VectorFloat sensorObservationVec;
		sensorInterface_->getCombinedObservation(sensorObservationVec);
		for (size_t i = 0; i != sensorObservationVec.size(); ++i) {
			observationVec[i + startIndex] = sensorObservationVec[i];
		}
	};

	getObservationFunctions_.push_back(sensorObservations);
}
}
