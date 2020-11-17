#include "GazeboInterface.hpp"

namespace oppt {
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

	for (auto &spaceVariable : orderedVariables->spaceVariables) {
		switch (spaceVariable) {
		case SpaceVariable::JOINT_POSITIONS:
			if (jointPositionVec.size() > 0) {
				SetStateFunction setPosition = [this, jointPositionVec, redundancyMapPosition, startIndex](const VectorFloat & stateVec) {
					VectorString redJoints;
					std::string redundantJoint;
					FloatType jaToSet = 0;
					JointMap jm = worldJointMap_[world_->GetName()];
					for (size_t i = 0; i != jointPositionVec.size(); i++) {
						JointPtr jp = jm[jointPositionVec[i]];
						jaToSet = stateVec[startIndex + i];
						jp->SetPosition(0, jaToSet);
						if (redundancyMapPosition.count(jointPositionVec[i]) == 1) {
							// Need to set the state of the redundant joints as well
							redJoints = redundancyMapPosition.at(jointPositionVec[i]);
							for (size_t j = 0; j < redJoints.size(); j++) {
								redundantJoint = redJoints[j];
								jm[redundantJoint]->SetPosition(0, jaToSet);
							}
						}
					}

				};

				GetStateFunction getPosition = [this, jointPositionVec, startIndex](VectorFloat & stateVec) {
					JointMap jm = worldJointMap_[world_->GetName()];
					for (size_t i = 0; i != jointPositionVec.size(); i++) {
						stateVec[startIndex + i] = jm[jointPositionVec[i]]->GetAngle(0).Radian();
					}
				};

				setPositionFunctions.push_back(setPosition);
				getStateFunctions_.push_back(getPosition);
				startIndex += jointPositionVec.size();
			}
			break;
		case SpaceVariable::JOINT_VELOCITIES:
			if (jointVelocityVec.size() > 0) {
				SetStateFunction setVelocity = [this, jointVelocityVec, redundancyMapVelocity, startIndex](const VectorFloat & stateVec) {
					VectorString redJoints;
					std::string redundantJoint;
					JointMap jm = worldJointMap_[world_->GetName()];
					for (size_t i = 0; i != jointVelocityVec.size(); i++) {
						setJointVelocityRecursive(jm[jointVelocityVec[i]].get(), stateVec[startIndex + i]);

						if (redundancyMapVelocity.count(jointVelocityVec[i]) == 1) {
							redJoints = redundancyMapVelocity.at(jointVelocityVec[i]);
							for (size_t j = 0; j < redJoints.size(); j++) {
								redundantJoint = redJoints[j];
								setJointVelocityRecursive(jm[redundantJoint].get(), stateVec[startIndex + i]);
							}
						}

					}
				};

				GetStateFunction getVelocity = [this, jointVelocityVec, startIndex](VectorFloat & stateVec) {
					JointMap jm = worldJointMap_[world_->GetName()];
					for (size_t i = 0; i < jointVelocityVec.size(); i++) {
						stateVec[startIndex + i] = jm[jointVelocityVec[i]]->GetVelocity(0);
					}
				};

				setVelocityFunctions.push_back(setVelocity);
				getStateFunctions_.push_back(getVelocity);
				startIndex += jointVelocityVec.size();
			}
			break;
		case SpaceVariable::LINK_POSES:
			if (linkPosesVec.size() > 0) {
				SetStateFunction setLinkPoses = [this, linkPosesVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPosesVec) {
						GZPose linkPose(stateVec[idx],
						                stateVec[idx + 1],
						                stateVec[idx + 2],
						                stateVec[idx + 3],
						                stateVec[idx + 4],
						                stateVec[idx + 5]);
						auto lmEntry = lm[link];
						lmEntry->SetWorldPose(linkPose);
						idx += 6;
					}
				};

				GetStateFunction getLinkPoses = [this, linkPosesVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPosesVec) {
						pose = lm[link]->GetWorldPose();
						GZVector3 orientation = pose.rot.GetAsEuler();
						stateVec[idx] = pose.pos.x;
						stateVec[idx + 1] = pose.pos.y;
						stateVec[idx + 2] = pose.pos.z;
						stateVec[idx + 3] = orientation.x;
						stateVec[idx + 4] = orientation.y;
						stateVec[idx + 5] = orientation.z;
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
				SetStateFunction setLinkPositionsX = [this, linkPositionsXVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPositionsXVec) {
						auto lmEntry = lm[link];
						auto currentPose = lmEntry->GetWorldPose();
						GZVector3 newPosition(stateVec[idx], currentPose.pos.y, currentPose.pos.z);
						currentPose.Set(newPosition, currentPose.rot);
						lmEntry->SetWorldPose(currentPose);
						idx += 1;
					}

				};

				GetStateFunction getLinkPositionsX = [this, linkPositionsXVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPositionsXVec) {
						pose = lm[link]->GetWorldPose();
						stateVec[idx] = pose.pos.x;
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
				SetStateFunction setLinkPositionsY = [this, linkPositionsYVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPositionsYVec) {
						auto lmEntry = lm[link];
						auto currentPose = lmEntry->GetWorldPose();
						GZVector3 newPosition(currentPose.pos.x, stateVec[idx], currentPose.pos.z);
						currentPose.Set(newPosition, currentPose.rot);
						lmEntry->SetWorldPose(currentPose);
						idx += 1;
					}

				};

				GetStateFunction getLinkPositionsY = [this, linkPositionsYVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPositionsYVec) {
						pose = lm[link]->GetWorldPose();
						stateVec[idx] = pose.pos.y;
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
				SetStateFunction setLinkPositionsZ = [this, linkPositionsZVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPositionsZVec) {
						auto lmEntry = lm[link];
						auto currentPose = lmEntry->GetWorldPose();
						GZVector3 newPosition(currentPose.pos.x, currentPose.pos.y, stateVec[idx]);
						currentPose.Set(newPosition, currentPose.rot);
						lmEntry->SetWorldPose(currentPose);
						idx += 1;
					}

				};

				GetStateFunction getLinkPositionsZ = [this, linkPositionsZVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkPositionsZVec) {
						pose = lm[link]->GetWorldPose();
						stateVec[idx] = pose.pos.z;
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
				SetStateFunction setLinkOrientationsX = [this, linkOrientationsXVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkOrientationsXVec) {
						auto lmEntry = lm[link];
						auto currentPose = lmEntry->GetWorldPose();
						auto currentEuler = currentPose.rot.GetAsEuler();
						GZVector3 newRot(stateVec[idx], currentEuler[1], currentEuler[2]);
						currentPose.Set(currentPose.pos, newRot);
						lmEntry->SetWorldPose(currentPose);
						idx += 1;
					}

				};

				GetStateFunction getLinkOrientationsX = [this, linkOrientationsXVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkOrientationsXVec) {
						pose = lm[link]->GetWorldPose();
						stateVec[idx] = pose.rot.GetAsEuler().x;
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
				SetStateFunction setLinkOrientationsY = [this, linkOrientationsYVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkOrientationsYVec) {
						auto lmEntry = lm[link];
						auto currentPose = lmEntry->GetWorldPose();
						auto currentEuler = currentPose.rot.GetAsEuler();
						GZVector3 newRot(currentEuler[0], stateVec[idx], currentEuler[2]);
						currentPose.Set(currentPose.pos, newRot);
						lmEntry->SetWorldPose(currentPose);
						idx += 1;
					}

				};

				GetStateFunction getLinkOrientationsY = [this, linkOrientationsYVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkOrientationsYVec) {
						pose = lm[link]->GetWorldPose();
						stateVec[idx] = pose.rot.GetAsEuler().y;
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
				SetStateFunction setLinkOrientationsZ = [this, linkOrientationsZVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkOrientationsZVec) {
						auto lmEntry = lm[link];
						auto currentPose = lmEntry->GetWorldPose();
						auto currentEuler = currentPose.rot.GetAsEuler();
						GZVector3 newRot(currentEuler[0], currentEuler[1], stateVec[idx]);
						currentPose.Set(currentPose.pos, newRot);
						lmEntry->SetWorldPose(currentPose);
						idx += 1;
					}

				};

				GetStateFunction getLinkOrientationsZ = [this, linkOrientationsZVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZPose pose;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkOrientationsZVec) {
						pose = lm[link]->GetWorldPose();
						stateVec[idx] = pose.rot.GetAsEuler().z;
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
				SetStateFunction setLinearVelocities = [this, linkVelocitiesLinearVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearVec) {
						GZVector3 vel(stateVec[idx], stateVec[idx + 1], stateVec[idx + 2]);
						lm[link]->SetLinearVel(vel);
						idx += 3;
					}
				};

				GetStateFunction getLinearVelocities = [this, linkVelocitiesLinearVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearVec) {
						vel = lm[link]->GetWorldLinearVel();
						stateVec[idx] = vel.x;
						stateVec[idx + 1] = vel.y;
						stateVec[idx + 2] = vel.z;
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
				SetStateFunction setAngularVelocities = [this, linkVelocitiesAngularVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularVec) {
						GZVector3 vel(stateVec[idx], stateVec[idx + 1], stateVec[idx + 2]);
						lm[link]->SetAngularVel(vel);
						idx += 3;
					}
				};

				GetStateFunction getAngularVelocities = [this, linkVelocitiesAngularVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularVec) {
						vel = lm[link]->GetWorldAngularVel();
						stateVec[idx] = vel.x;
						stateVec[idx + 1] = vel.y;
						stateVec[idx + 2] = vel.z;
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
				SetStateFunction setLinearVelocitiesX = [this, linkVelocitiesLinearXVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearXVec) {
						GZVector3 currentVel = lm[link]->GetWorldLinearVel();
						GZVector3 vel(stateVec[idx], currentVel.y, currentVel.z);
						lm[link]->SetLinearVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getLinearVelocitiesX = [this, linkVelocitiesLinearXVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearXVec) {
						vel = lm[link]->GetWorldLinearVel();
						stateVec[idx] = vel.x;
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
				SetStateFunction setLinearVelocitiesY = [this, linkVelocitiesLinearYVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearYVec) {
						GZVector3 currentVel = lm[link]->GetWorldLinearVel();
						GZVector3 vel(currentVel.x, stateVec[idx], currentVel.z);
						lm[link]->SetLinearVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getLinearVelocitiesY = [this, linkVelocitiesLinearYVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearYVec) {
						vel = lm[link]->GetWorldLinearVel();
						stateVec[idx] = vel.y;
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
				SetStateFunction setLinearVelocitiesZ = [this, linkVelocitiesLinearZVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearZVec) {
						GZVector3 currentVel = lm[link]->GetWorldLinearVel();
						GZVector3 vel(currentVel.x, currentVel.y, stateVec[idx]);
						lm[link]->SetLinearVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getLinearVelocitiesZ = [this, linkVelocitiesLinearZVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesLinearZVec) {
						vel = lm[link]->GetWorldLinearVel();
						stateVec[idx] = vel.z;
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
				SetStateFunction setAngularVelocitiesX = [this, linkVelocitiesAngularXVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularXVec) {
						GZVector3 currentVel = lm[link]->GetWorldAngularVel();
						GZVector3 vel(stateVec[idx], currentVel.y, currentVel.z);
						lm[link]->SetAngularVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getAngularVelocitiesX = [this, linkVelocitiesAngularXVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularXVec) {
						vel = lm[link]->GetWorldAngularVel();
						stateVec[idx] = vel.x;
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
				SetStateFunction setAngularVelocitiesY = [this, linkVelocitiesAngularYVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularYVec) {
						GZVector3 currentVel = lm[link]->GetWorldAngularVel();
						GZVector3 vel(currentVel.x, stateVec[idx], currentVel.z);
						lm[link]->SetAngularVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getAngularVelocitiesY = [this, linkVelocitiesAngularYVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularYVec) {
						vel = lm[link]->GetWorldAngularVel();
						stateVec[idx] = vel.y;
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
				SetStateFunction setAngularVelocitiesZ = [this, linkVelocitiesAngularZVec, startIndex](const VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularZVec) {
						GZVector3 currentVel = lm[link]->GetWorldAngularVel();
						GZVector3 vel(currentVel.x, currentVel.y, stateVec[idx]);
						lm[link]->SetAngularVel(vel);
						idx += 1;
					}
				};

				GetStateFunction getAngularVelocitiesZ = [this, linkVelocitiesAngularZVec, startIndex](VectorFloat & stateVec) {
					unsigned int idx = startIndex;
					GZVector3 vel;
					LinkMap lm = worldLinkMap_[world_->GetName()];
					for (auto & link : linkVelocitiesAngularZVec) {
						vel = lm[link]->GetWorldAngularVel();
						stateVec[idx] = vel.z;
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

    for (auto &spaceVariable : orderedVariables->spaceVariables) {
        switch (spaceVariable) {
        case SpaceVariable::JOINT_POSITIONS:
            if (observedJointPositions.size() > 0) {
                GetObservationFunction obsFunct = [this,
                                                   observedJointPositions,
                startIndex](VectorFloat & observationVec) {
                    JointMap jm = worldJointMap_[world_->GetName()];
                    for (size_t i = 0; i != observedJointPositions.size(); i++) {
                        observationVec[startIndex + i] = jm[observedJointPositions[i]]->GetAngle(0).Radian();
                    }
                };

                getObservationFunctions_.push_back(obsFunct);
                startIndex += observedJointPositions.size();
            }
            break;
        case SpaceVariable::JOINT_VELOCITIES:
            if (observedJointVelocities.size() > 0) {
                GetObservationFunction obsFunct = [this,
                                                   observedJointVelocities,
                startIndex](VectorFloat & observationVec) {
                    JointMap jm = worldJointMap_[world_->GetName()];
                    for (size_t i = 0; i != observedJointVelocities.size(); i++) {
                        observationVec[startIndex + i] = jm[observedJointVelocities[i]]->GetVelocity(0);
                    }
                };

                getObservationFunctions_.push_back(obsFunct);
                startIndex += observedJointVelocities.size();
            }
            break;
        case SpaceVariable::LINK_POSES:
            if (observedLinkPoses.size() > 0) {
                GetObservationFunction obsFunct = [this,
                                                   observedLinkPoses,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZPose linkPose;
                    GZVector3 rotation;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkPoses) {
                        linkPose = lm[link]->GetWorldPose();
                        rotation = linkPose.rot.GetAsEuler();
                        observationVec[idx] = linkPose.pos.x;
                        observationVec[idx + 1] = linkPose.pos.y;
                        observationVec[idx + 2] = linkPose.pos.z;
                        observationVec[idx + 3] = rotation.x;
                        observationVec[idx + 4] = rotation.y;
                        observationVec[idx + 5] = rotation.z;
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
                                                   observedLinkPositionsX,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto &link : observedLinkPositionsX) {
                        observationVec[idx] = lm[link]->GetWorldPose().pos.x;
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
                                                   observedLinkPositionsY,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto &link : observedLinkPositionsY) {
                        observationVec[idx] = lm[link]->GetWorldPose().pos.y;
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
                                                   observedLinkPositionsZ,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto &link : observedLinkPositionsZ) {
                        observationVec[idx] = lm[link]->GetWorldPose().pos.z;
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
                                                   observedLinkVelocitiesLinear,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesLinear) {
                        vel = lm[link]->GetWorldLinearVel();
                        observationVec[idx] = vel.x;
                        observationVec[idx + 1] = vel.y;
                        observationVec[idx + 2] = vel.z;
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
                                                   observedLinkVelocitiesAngular,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesAngular) {
                        vel = lm[link]->GetWorldAngularVel();
                        observationVec[idx] = vel.x;
                        observationVec[idx + 1] = vel.y;
                        observationVec[idx + 2] = vel.z;
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
                                                   observedLinkVelocitiesLinearX,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesLinearX) {
                        vel = lm[link]->GetWorldLinearVel();
                        observationVec[idx] = vel.x;
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
                                                   observedLinkVelocitiesLinearY,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesLinearY) {
                        vel = lm[link]->GetWorldLinearVel();
                        observationVec[idx] = vel.y;
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
                                                   observedLinkVelocitiesLinearZ,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesLinearZ) {
                        vel = lm[link]->GetWorldLinearVel();
                        observationVec[idx] = vel.z;
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
                                                   observedLinkVelocitiesAngularX,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesAngularX) {
                        vel = lm[link]->GetWorldAngularVel();
                        observationVec[idx] = vel.x;
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
                                                   observedLinkVelocitiesAngularY,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesAngularY) {
                        vel = lm[link]->GetWorldAngularVel();
                        observationVec[idx] = vel.y;
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
                                                   observedLinkVelocitiesAngularZ,
                startIndex](VectorFloat & observationVec) {
                    unsigned int idx = startIndex;
                    GZVector3 vel;
                    LinkMap lm = worldLinkMap_[world_->GetName()];
                    for (auto & link : observedLinkVelocitiesAngularZ) {
                        vel = lm[link]->GetWorldAngularVel();
                        observationVec[idx] = vel.z;
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
