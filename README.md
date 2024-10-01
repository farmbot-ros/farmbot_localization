# farmbot_localization

[![MIT License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview
`farmbot_localization` is a ROS package designed to provide localization capabilities for the FarmBot project. This repository is part of the `farmbot-ros` organization and focuses on enhancing the precision and efficiency of FarmBot's operations through advanced localization techniques.

## Repository Structure
- **C++**: 90.1%
- **Python**: 6.1%
- **CMake**: 3.8%

## Getting Started
To get started with `farmbot_localization`, clone the repository and follow the setup instructions provided below.

```bash
git clone https://github.com/farmbot-ros/farmbot_localization.git
cd farmbot_localization
```

## Installation
Detailed installation instructions will be provided here. For now, please follow the standard ROS package installation procedures.

## Usage
Instructions on how to use the package and integrate it with other FarmBot systems will be provided here.

## Internals
### Overview
The `farmbot_localization` package provides advanced localization capabilities for the FarmBot project, utilizing various localization methods including single antenna, dual antenna, and bearing-based localization. This package allows for flexible configuration through parameters set in Python files, and the datum can be set either automatically or manually.

### Localization Methods
1. **Single Antenna (`src/single_antenna.cpp`)**
   - Uses a Kalman Filter to process GPS data.
   - Configures topics for GPS data and publishes front and back antenna data.
   - Parameters: `gps_topic`, `gps_main`, `gps_aux`, `kallman_type`.

2. **Dual Antenna (`src/dual_antenna.cpp`)**
   - Synchronizes data from dual antennas to enhance localization accuracy.
   - Calculates bearing between GPS points.
   - Parameters: `gps_main`, `gps_aux`, `angle_gpses`.

3. **Bearing-Based Localization (`src/fix_n_bearing.cpp`)**
   - Processes GPS and angle degree data to publish localized fixes.
   - Converts angles to radians for calculations.
   - Parameters: `gps_corr`, `angle_deg`.

### Datum Settings
- **Automatic Datum Setting** (`src/using_enu.cpp`)
  - Automatically sets the datum based on GPS lock time.
  - Parameters: `autodatum`.

- **Manual Datum Setting**
  - Datum can be set manually through service calls.
  - Services: `farmbot_interfaces/srv/Datum`, `farmbot_interfaces/srv/Trigger`.

### Parameters
- **Thresholds and Filters**
  - Kalman Filter parameters: `process_noise`, `measurement_noise`.
  - Movement threshold for switching modes: `threshold`.

- **Topic Prefixes**
  - Customizable topic prefixes for publishing and subscribing to ROS topics.
  - Parameters: `topic_prefix`.

### Key Classes and Methods

1. **GpsAndDEg**
   - Handles GPS and degree calculations.
   - Methods: `gps_corr_callback`, `angle_deg_callback`, `timer_callback`.

2. **AntennaSplit**
   - Manages split of GPS data for single antenna mode.
   - Methods: `callback`, `thresh_callback`.

3. **AntennaFuse**
   - Fuses data from dual antennas for enhanced localization.
   - Methods: `callback`, `calc_bearing`.

4. **Gps2Enu**
   - Converts GPS data to ENU (East-North-Up) coordinate system.
   - Methods: `callback`, `datum_gps_callback`, `datum_set_callback`.

## Contributing
Contributions are welcome! Please fork the repository and submit pull requests. Make sure to follow the contribution guidelines.

## License
This project is licensed under the MIT License - see the [LICENSE](https://github.com/farmbot-ros/farmbot_localization/blob/develop/LICENSE) file for details.

## Contact
For any inquiries or support, please open an issue on the repository.
