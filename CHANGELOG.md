# Changelog

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