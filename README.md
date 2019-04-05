# <img src="resource/BSDFProcessor.png" width="48"/> BSDF Processor
BSDF Processor is an application for interactive visualization of BSDF (bidirectional scattering distribution function).
The user can investigate and edit BRDF (bidirectional reflectance distribution function) and BTDF (bidirectional transmittance distribution function).
Basic functions are implemented using [libbsdf][1].

## Features
* 3D plot using incoming angles and multiple display modes
* Real-time rendering with directional and environment lighting
* Table view of sample points
* Simple editor
* File loaders for multiple formats
* BRDF/BTDF generators using reflectance/transmittance models

**BSDF Processor can load the following formats:**

Format | Extension | Measured Data |
-------|---------------------------------|-----|
Integra Diffuse Distribution | .ddr, .ddt |   |
Integra Specular Distribution | .sdr, .sdt |   |
Zemax BSDF | .bsdf | [RPC Photonics][9] |
LightTools BSDF | .bsdf |   |
ASTM E1392-96(2002) | .astm | [Cornell University][7] |
MERL BRDF | .binary | [MERL][8] |

**BSDF Processor can export the following formats:**

Format | Extension
-------|---------------------------------
Integra Diffuse Distribution | .ddr, .ddt

## Gallery
#### Screenshots
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/KimuraRyo/BSDFProcessor/images/screenshot1.png" height="190"/>&nbsp;
<img src="https://raw.githubusercontent.com/wiki/KimuraRyo/BSDFProcessor/images/screenshot2.png" height="190"/>&nbsp;
<img src="https://raw.githubusercontent.com/wiki/KimuraRyo/BSDFProcessor/images/screenshot3.png" height="190"/>
</p>

#### Movie
[![mov](http://img.youtube.com/vi/AJXpTs0InYc/mqdefault.jpg)](https://youtu.be/AJXpTs0InYc)

## License
BSDF Processor is licensed under the terms of the Mozilla Public License, version 2.0.
See the LICENSE file.

## Download
Windows (64bit):

[BSDFProcessor-1.2.1-windows-x64.zip][6]

## Building BSDF Processor
BSDF Processor uses the following open source libraries:

* [libbsdf][1]
* [Eigen 3][2]
* [OpenSceneGraph][3] ([osgQt][10] is required for OpenSceneGraph 3.6 or higher.)
* [Qt 5][4]

CMake is used as the build system.
Search paths for libraries are set through CMake variables: `LIBBSDF_DIR`, `OSG_DIR`, `EIGEN3_INCLUDE_DIR`, and `CMAKE_PREFIX_PATH` for Qt 5.

## Sponsor
[<img src="resource/IntegraLogo.png" width="200"/>][5]

[1]: https://github.com/KimuraRyo/libbsdf "libbsdf"
[2]: http://eigen.tuxfamily.org/index.php?title=Main_Page "Eigen"
[3]: http://www.openscenegraph.org "OpenSceneGraph"
[4]: http://www.qt.io "Qt"
[5]: http://www.integra.jp/en "Integra"
[6]: https://raw.githubusercontent.com/wiki/KimuraRyo/BSDFProcessor/binaries/BSDFProcessor-1.2.1-windows-x64.zip
[7]: http://www.graphics.cornell.edu/online/measurements/reflectance/
[8]: http://www.merl.com/brdf
[9]: http://www.rpcphotonics.com/bsdf-data-optical-diffusers/
[10]: https://github.com/openscenegraph/osgQt
