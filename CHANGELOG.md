# Changelog

## [1.2.0] 2024-08-13

### Added

- Add an option to build the TWR_Demo project using UWB channel 5 or 9
- Add default TX power values for channel 5 and 9.

### Changed

- During initialization TX power and antenna delay values are now fetched from QM33110W's OTP memory instead of a fix value.
- Minor code improvements

### Fixed

- Increase lisde12 power delay. The previous delay was not enough for the device to properly boots.

## [1.1.0] 2024-07-10

### Changed

- Replace accelerometer implementation with a one based of the lis2de12 repository from ST
- Implement activity detection

### Fixed

- Fix an issue where the accelerometer was drawing too much current

## [1.0.2] 2024-04-22

### Fixed

- Fix UWB frame filtering
- Add timeout to uwb_wake_up() waiting loop (to avoid being stuck in infinite loop)
- Fix pin assignement & address for LIS2DE12

## [1.0.1] 2024-01-09

### Fixed

- Add missing *.a library files in the project


## [1.0.0] 2023-11-28

- Initial version.