// =================================================================== //
// Copyright (C) 2015-2020 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

/*!
 * \file    Version.h
 * \brief   The version number of BSDF Processor.
 */

#ifndef VERSION_H
#define VERSION_H

#define BSDFPROCESSOR_MAJOR_VERSION 1
#define BSDFPROCESSOR_MINOR_VERSION 2
#define BSDFPROCESSOR_PATCH_VERSION 4

/*! Returns the version number of BSDF Processor. */
const char* getVersion();

#endif // VERSION_H
