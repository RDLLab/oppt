#ifndef _OPPT_SPACE_COMPONENTS_HPP_
#define _OPPT_SPACE_COMPONENTS_HPP_
#include "typedefs.hpp"

namespace oppt {
/** @brief Different variables a space can consist of */
enum SpaceVariable {JOINT_POSITIONS,
                    JOINT_POSITIONS_INCREMENT,
                    JOINT_VELOCITIES,
                    JOINT_TORQUES,
                    LINK_POSES,
                    LINK_POSITIONS_X,
                    LINK_POSITIONS_Y,
                    LINK_POSITIONS_Z,
                    LINK_ORIENTATIONS_X,
                    LINK_ORIENTATIONS_Y,
                    LINK_ORIENTATIONS_Z,
                    LINK_VELOCITIES_LINEAR,
                    LINK_VELOCITIES_ANGULAR,
                    LINK_VELOCITIES_LINEAR_X,
                    LINK_VELOCITIES_LINEAR_Y,
                    LINK_VELOCITIES_LINEAR_Z,
                    LINK_VELOCITIES_ANGULAR_X,
                    LINK_VELOCITIES_ANGULAR_Y,
                    LINK_VELOCITIES_ANGULAR_Z,
                    USER_DEFINED
                   };

/** @brief Data structure containing a vector of oppt::SpaceVariable */
struct SpaceVariables {
	std::vector<SpaceVariable> spaceVariables;
};

/** @brief std::unique_ptr to oppt::SpaceVariables*/
typedef std::shared_ptr<SpaceVariables> SpaceVariablesPtr;

struct VectorSpaceVariables: public SpaceVariables {
	VectorUInt indices;
};

}

#endif