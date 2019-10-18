// =================================================================== //
// Copyright (C) 2014-2019 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include "SpecularCenteredCoordinateSystem.h"

using namespace lb;

const char SpecularCenteredCoordinateSystem::ANGLE0_NAME[] = "Incoming polar angle";
const char SpecularCenteredCoordinateSystem::ANGLE1_NAME[] = "Incoming azimuthal angle";
const char SpecularCenteredCoordinateSystem::ANGLE2_NAME[] = "Specular polar angle";
const char SpecularCenteredCoordinateSystem::ANGLE3_NAME[] = "Specular azimuthal angle";

const float SpecularCenteredCoordinateSystem::MAX_ANGLE0 = lb::decrease(lb::PI_2_F);
const float SpecularCenteredCoordinateSystem::MAX_ANGLE1 = lb::decrease(lb::TAU_F);
const float SpecularCenteredCoordinateSystem::MAX_ANGLE2 = lb::decrease(lb::PI_2_F);
const float SpecularCenteredCoordinateSystem::MAX_ANGLE3 = lb::decrease(lb::TAU_F);
