# BSDF Viewer
## Overview
BSDF Viewer is an application for interactive visualization of BSDF (bidirectional scattering distribution function).
The target users of BSDF Viewer are engineers and researchers investigating the measured reflection and transmission data.
BSDF Viewer can load data files and provides 3D plots, table view and so on.
Basic functions are implemented using [libbsdf][1].

**BSDF Viewer can load the following formats:**

Format | Extension
-------|---------------------------------
Integra Diffuse Distribution | .ddr, .ddt
Integra Specular Distribution | .sdr, .sdt
LightTools BSDF | .bsdf
ASTM E1392-96(2002) | .astm
MERL BRDF | .binary

**BSDF Viewer can export the following formats:**

Format | Extension
-------|---------------------------------
Integra Diffuse Distribution | .ddr, .ddt

### License
BSDF Viewer is licensed under the terms of the Mozilla Public License, version 2.0.
See LICENSE file.

## Building BSDF Viewer
BSDF Viewer uses the following open source libraries:

* [libbsdf][1]
* [Eigen 3][2]
* [OpenSceneGraph][3]
* [Qt 5][4]

CMake is used as the build system.
Search paths for libraries are set through CMake variables: `LIBBSDF_DIR`, `OSG_DIR`, `EIGEN3_INCLUDE_DIR`, and `CMAKE_PREFIX_PATH` for Qt 5.

## Future Plans
* Loads other file formats
* Displays anisotropic BSDFs
* Adds spline or rational interpolation

## Sponsor
[[[6]|width=200px]]

[1]: https://github.com/KimuraRyo/libbsdf "libbsdf"
[2]: http://eigen.tuxfamily.org/index.php?title=Main_Page "Eigen"
[3]: http://www.openscenegraph.org "OpenSceneGraph"
[4]: http://www.qt.io "Qt"
[5]: http://www.integra.jp/en "Integra"
[6]: https://raw.githubusercontent.com/KimuraRyo/BSDFViewer/master/resource/IntegraLogo.png "IntegraLogo"
