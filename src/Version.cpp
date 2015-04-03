// =================================================================== //
// Copyright (C) 2014-2015 Kimura Ryo                                  //
//                                                                     //
// This Source Code Form is subject to the terms of the Mozilla Public //
// License, v. 2.0. If a copy of the MPL was not distributed with this //
// file, You can obtain one at http://mozilla.org/MPL/2.0/.            //
// =================================================================== //

#include <Version.h>

#include <stdio.h>

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

const char* getVersion()
{
    static bool initialized = false;
    static char version[256];

    if (!initialized) {
        sprintf(version, "%d.%d.%d", BSDFVIEWER_MAJOR_VERSION, BSDFVIEWER_MINOR_VERSION, BSDFVIEWER_PATCH_VERSION);
        initialized = true;
    }
    
    return version;
}
