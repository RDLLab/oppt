/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_ODE_STEP_BULLET_INC_H_
#define _GAZEBO_ODE_STEP_BULLET_INC_H_

#ifdef HAVE_BULLET
// This disable warning messages
#pragma GCC system_header
#include "LinearMath/btMatrixX.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#ifdef LIBBULLET_VERSION_GT_282
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#endif
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#endif
#endif

