# Changelog

## [1.2.2] - 2019-09-13

### Added
- Applied the logger of libbsdf.
- Added a switching preprocessor (LIBBSDF_DOUBLE_PRECISION_VECTOR) for double precision vectors.

### Changed
- Dealt with libbsdf-0.9.12.
- Improved the BTDF generator for the refractive index of less than 1.
- Adjusted the initial values of parameters for BRDF/BTDF generators.
- Improved 3D plot of anisotropic data with an incoming polar angle of zero.
- Adjusted the maximum lengths of directions and axes.
- Increased the maximum value of parameters for BRDF/BTDF generators.
- Adjusted intervals of vertices of 3D plot.
- Replaced the reflectance computation using Monte Carlo integration with steradian-based integration.
- Applied the constructor of BRDF/BTDF with narrow intervals near specular directions.

### Fixed
- Fixed the click on rendering view without data.
- Fixed the bug of initial angle display with a nonzero minimum angle.

[1.2.2]: https://github.com/KimuraRyo/BSDFProcessor/compare/v1.2.1...v1.2.2
