// =================================================================== //
// Copyright (C) 2014-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "SpecularCenteredCoordinateSystem.h"

using namespace lb;

const char SpecularCenteredCoordinateSystem::ANGLE0_NAME[] = "incoming polar angle";
const char SpecularCenteredCoordinateSystem::ANGLE1_NAME[] = "incoming azimuthal angle";
const char SpecularCenteredCoordinateSystem::ANGLE2_NAME[] = "specular polar angle";
const char SpecularCenteredCoordinateSystem::ANGLE3_NAME[] = "specular azimuthal angle";

const float SpecularCenteredCoordinateSystem::MAX_ANGLE0 = lb::PI_2_F;
const float SpecularCenteredCoordinateSystem::MAX_ANGLE1 = lb::TAU_F;
const float SpecularCenteredCoordinateSystem::MAX_ANGLE2 = lb::PI_2_F;
const float SpecularCenteredCoordinateSystem::MAX_ANGLE3 = lb::TAU_F;
