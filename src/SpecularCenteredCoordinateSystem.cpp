// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "SpecularCenteredCoordinateSystem.h"

using namespace lb;

const std::string SpecularCenteredCoordinateSystem::ANGLE0_NAME = "Incoming polar angle";
const std::string SpecularCenteredCoordinateSystem::ANGLE1_NAME = "Incoming azimuthal angle";
const std::string SpecularCenteredCoordinateSystem::ANGLE2_NAME = "Specular polar angle";
const std::string SpecularCenteredCoordinateSystem::ANGLE3_NAME = "Specular azimuthal angle";

const float SpecularCenteredCoordinateSystem::MAX_ANGLE0 = PI_2_F -      EPSILON_F * PI_2_F;
const float SpecularCenteredCoordinateSystem::MAX_ANGLE1 = 2.0f * PI_F - EPSILON_F * 2.0f * PI_F;
const float SpecularCenteredCoordinateSystem::MAX_ANGLE2 = PI_2_F -      EPSILON_F * PI_2_F;
const float SpecularCenteredCoordinateSystem::MAX_ANGLE3 = 2.0f * PI_F - EPSILON_F * 2.0f * PI_F;
