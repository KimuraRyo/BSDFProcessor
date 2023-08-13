# Changelog

## [1.3.0] - 2023-08-13

### Changed

- Updated code to support libbsdf-0.11.0.
- Changed 32-bit variables for processing to 64-bit.

## [1.2.6] - 2022-11-06

### Added

- Added the view of 2D scattered data for developers.

### Changed

- Updated code to support libbsdf-0.10.1.

## [1.2.5] - 2020-09-13

### Added

- Added BRDF/BTDF generation with the adjested intervals of angles in the "Reflectance/Transmittance" dock widget.

## [1.2.4] - 2020-08-09

### Changed

- Changed tangent directions of circular faces of cylinder in the render view for anisotropic BRDF.

### Fixed

- Improved the precision of incoming and outgoing directions in the render view.

## [1.2.3] - 2020-08-02

### Added

- Angles of the picked direction in the 3D graph, render view, and table view are displayed in the "Picked direction" dock widget.
- "Property view" dock widget to display properties of data.
- "Characteristic view" dock widget to display characteristics of data.
- Saving and restoring of application settings with QSettings.
- Loading and saving of SSDD file for BRDF/BTDF/specular reflectance/specular transmittance.

### Changed

- Dealt with libbsdf-0.9.13.
- Changed the picking of incoming direction to a single click.
- Improved high DPI settings.
- Renamed dock widgets.
- Adjusted item layout and text labels of UI.

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

[1.3.0]: https://github.com/KimuraRyo/BSDFProcessor/compare/v1.2.6...v1.3.0
[1.2.6]: https://github.com/KimuraRyo/BSDFProcessor/compare/v1.2.5...v1.2.6
[1.2.5]: https://github.com/KimuraRyo/BSDFProcessor/compare/v1.2.4...v1.2.5
[1.2.4]: https://github.com/KimuraRyo/BSDFProcessor/compare/v1.2.3...v1.2.4
[1.2.3]: https://github.com/KimuraRyo/BSDFProcessor/compare/v1.2.2...v1.2.3
[1.2.2]: https://github.com/KimuraRyo/BSDFProcessor/compare/v1.2.1...v1.2.2
