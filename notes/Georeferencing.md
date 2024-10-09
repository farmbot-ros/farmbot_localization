# Georeferencing in a Robot Navigation System: WGS84 to Local Coordinates

Georeferencing in a robot navigation system involves transforming global positioning data (such as GPS coordinates) into a local coordinate frame that the robot can use for accurate movement, path planning, and localization within its immediate environment. This process is crucial for ensuring that the robot maintains an accurate understanding of its position relative to the physical world, especially in outdoor or large-scale environments.

## Table of Contents
1. [Introduction](#introduction)
2. [Core Concepts](#core-concepts)
   - [Coordinate Reference Systems (CRS) and Georeferencing](#coordinate-reference-systems-crs-and-georeferencing)
   - [Integrating GPS and Local Navigation](#integrating-gps-and-local-navigation)
   - [Handling Global Coordinates in ROS 2](#handling-global-coordinates-in-ros-2)
3. [Theoretical Background: Georeferencing and Coordinate Systems](#theoretical-background-georeferencing-and-coordinate-systems)
   - [Understanding Ellipsoids and Datums](#understanding-ellipsoids-and-datums)
   - [Geographic Coordinate System (GCS)](#geographic-coordinate-system-gcs)
   - [Geocentric Coordinate System (Geocentric CRS)](#geocentric-coordinate-system-geocentric-crs)
   - [Projected Coordinate Systems (PCS)](#projected-coordinate-systems-pcs)
   - [Well-Known Text (WKT) and EPSG Codes](#well-known-text-wkt-and-epsg-codes)
   - [Importance of Choosing the Right CRS](#importance-of-choosing-the-right-crs)
   - [Coordinate Frames in ROS 2](#coordinate-frames-in-ros-2)
4. [Transformation Processes](#transformation-processes)
   - [WGS84 to ECEF to ENU](#wgs84-to-ecef-to-enu)
   - [WGS84 to UTM](#wgs84-to-utm)
8. [Conclusion](#conclusion)

---

## Introduction

In robot navigation systems, especially those operating in outdoor environments, accurate positioning is paramount. **Georeferencing** bridges the gap between global positioning data (like GPS coordinates) and the robot's local coordinate system, enabling precise navigation and localization. By converting global coordinates into a local frame, robots can effectively navigate, plan paths, and perform tasks with high accuracy, mitigating issues like odometry drift.

This document delves into the theoretical and practical aspects of georeferencing within a ROS 2-based robot navigation system. It covers the foundational concepts of Coordinate Reference Systems (CRS), the transformation processes from global to local coordinates, and the implementation of these transformations using ROS 2 tools and packages.

---

## Core Concepts

### Coordinate Reference Systems (CRS) and Georeferencing

A **Coordinate Reference System (CRS)** defines how spatial data is mapped to coordinates on the Earth's surface. **Georeferencing** is the process of aligning spatial data (such as a map or a robot's sensor data) with a CRS to ensure accurate positioning.

### Integrating GPS and Local Navigation

Integrating GPS data with local navigation systems involves transforming global coordinates (latitude, longitude, altitude) into a local frame of reference that the robot can use for immediate navigation tasks. This integration helps in correcting drift and maintaining accurate localization over time.

### Handling Global Coordinates in ROS 2

In ROS 2, handling global coordinates involves using transformation libraries like **tf2** to manage and convert between different coordinate frames. This allows the robot to understand its position in both global and local contexts, facilitating accurate navigation and task execution.

---

## Theoretical Background: Georeferencing and Coordinate Systems

Understanding georeferencing in robotic systems requires a solid grasp of various **Coordinate Reference Systems (CRS)** and the mathematical models that underpin them. This section delves into the theoretical aspects of georeferencing, focusing on CRS, ellipsoids, and the transformations necessary for accurate robotic navigation.

### Understanding Ellipsoids and Datums

#### Ellipsoids

Planets like Earth are not perfect spheres; they are **ellipsoids**, meaning they are slightly flattened at the poles and bulging at the equator. This shape is mathematically modeled to accurately represent the Earth's surface for mapping and navigation purposes.

#### Datums

A **datum** defines the size and shape of the ellipsoid and its orientation relative to the Earth's center. The most widely used datum is **WGS84 (World Geodetic System 1984)**, which serves as the standard for GPS systems.

**WGS84 Ellipsoid Parameters:**
- **Semi-major axis (a)**: 6,378,137.0 meters
- **Semi-minor axis (b)**: 6,356,752.314245 meters
- **Flattening (1/f)**: 298.257223563

These parameters define the ellipsoid's shape, providing the foundation for accurate geospatial measurements.

### Geographic Coordinate System (GCS)

A **Geographic Coordinate System (GCS)** uses spherical coordinates—**latitude**, **longitude**, and **altitude**—to specify locations on the Earth's surface.

- **Latitude**: Measures north-south position between the poles.
- **Longitude**: Measures east-west position relative to the Prime Meridian.
- **Altitude**: Measures height above or below the reference ellipsoid.

**Representation:**
- Angles can be expressed in **decimal degrees** or **Degrees, Minutes, Seconds (DMS)**.

**Note:** When the datum is not a perfect sphere, calculating positions involves more complex computations to account for the ellipsoid's shape.

### Geocentric Coordinate System (Geocentric CRS)

A **Geocentric CRS** employs a 3D Cartesian coordinate system centered at the Earth's center.

- **Origin**: Earth's center.
- **X-axis**: Points towards the Equator/Prime Meridian intersection.
- **Y-axis**: Orthogonal to the X-axis, pointing 90° east.
- **Z-axis**: Aligns with the Earth's rotational axis towards the North Pole.

Coordinates in this system are expressed in meters and are essential for applications requiring precise global positioning, such as distributed simulations (e.g., DIS/HLA protocols).

### Projected Coordinate Systems (PCS)

A **Projected Coordinate System (PCS)** converts the Earth's curved surface into a flat, two-dimensional plane using map projections. This transformation simplifies distance and area calculations, making it easier to work with spatial data on maps.

#### Types of Projections

- **Planar**: Projects the Earth onto a plane, suitable for small areas.
- **Conical**: Projects the Earth onto a cone, useful for mid-latitude regions.
- **Cylindrical**: Projects the Earth onto a cylinder, commonly used for global maps.

#### Universal Transverse Mercator (UTM)

One of the most widely used PCS is the **Universal Transverse Mercator (UTM)** system, which divides the Earth into 60 zones, each spanning 6° of longitude. Each zone uses a transverse Mercator projection, providing accurate local measurements in meters.

**UTM Features:**
- **Eastings**: X-coordinate, representing distance east from the central meridian (500,000 meters to avoid negative values).
- **Northings**: Y-coordinate, representing distance north from the equator (0 meters at the equator in the Northern Hemisphere; 10,000,000 meters in the Southern Hemisphere to maintain positive values).
- **Zones**: Each 6° wide, numbered 1 to 60, covering the globe longitudinally.

**Derivatives:** Military Grid Reference System (MGRS) is a derivative of UTM, used extensively in military applications.

### Well-Known Text (WKT) and EPSG Codes

To describe a CRS, standardized formats like **Well-Known Text (WKT)** and **European Petroleum Survey Group (EPSG)** codes are used. These formats encapsulate all necessary parameters, such as ellipsoid, units, and projection details, enabling interoperability across different software and systems.

**EPSG Codes:**
- Unique identifiers for specific CRS definitions.
- Example: EPSG:4326 refers to WGS84.

**Importance:**
- Ensures consistent CRS usage across different platforms and applications.
- Facilitates accurate coordinate transformations and data integration.

### Importance of Choosing the Right CRS

Selecting an appropriate CRS is critical for accurate georeferencing. The chosen CRS must align with the application's spatial requirements, considering factors like geographic extent, required precision, and compatibility with existing data.

---

### Coordinate Frames in ROS 2

In ROS 2, coordinate frames are essential for defining the spatial relationships between different parts of the robot and its environment. Adhering to standardized conventions ensures consistency and interoperability across various software components and hardware platforms.

#### Base Link (`base_link`)

The coordinate frame called `base_link` is rigidly attached to the mobile robot base. The `base_link` can be attached to the base in any arbitrary position or orientation; for every hardware platform, there will be a different place on the base that provides an obvious point of reference. Note that REP 103 specifies a preferred orientation for frames.

#### Odometry Frame (`odom`)

The coordinate frame called `odom` is a world-fixed frame. The pose of a mobile platform in the `odom` frame can drift over time without any bounds. This drift makes the `odom` frame unsuitable as a long-term global reference. However, the pose of a robot in the `odom` frame is guaranteed to be continuous, meaning that the pose of a mobile platform in the `odom` frame always evolves smoothly, without discrete jumps.

In a typical setup, the `odom` frame is computed based on an odometry source, such as wheel odometry, visual odometry, or an inertial measurement unit.

The `odom` frame is useful as an accurate, short-term local reference, but drift makes it a poor frame for long-term reference.

#### Map Frame (`map`)

The coordinate frame called `map` is a world-fixed frame, with its Z-axis pointing upwards. The pose of a mobile platform relative to the `map` frame should not significantly drift over time. The `map` frame is not continuous, meaning the pose of a mobile platform in the `map` frame can change in discrete jumps at any time.

In a typical setup, a localization component constantly re-computes the robot pose in the `map` frame based on sensor observations, thereby eliminating drift but causing discrete jumps when new sensor information arrives.

The `map` frame is useful as a long-term global reference, but discrete jumps in position estimators make it a poor reference frame for local sensing and acting.

#### Earth Frame (`earth`)

The coordinate frame called `earth` is the origin of ECEF. This frame is designed to allow the interaction of multiple robots in different map frames. If the application only needs one map, the `earth` coordinate frame is not expected to be present. In the case of running with multiple maps simultaneously, the `map` and `odom` and `base_link` frames will need to be customized for each robot. If running multiple robots and bridging data between them, the transform `frame_ids` can remain standard on each robot if the other robots' `frame_ids` are rewritten.

If the `map` frame is globally referenced, the publisher from `earth` to `map` can be a static transform publisher. Otherwise, the `earth` to `map` transform will usually need to be computed by taking the estimate of the current global position and subtracting the current estimated pose in the `map` to get the estimated pose of the origin of the `map`.

In case the `map` frame's absolute position is unknown at the time of startup, it can remain detached until such time that the global position estimation can be adequately evaluated. This will operate in the same way that a robot can operate in the `odom` frame before localization in the `map` frame is initialized.

---

### Map Conventions

Map coordinate frames can either be referenced globally or to an application-specific position. An example of an application-specific positioning might be Mean Sea Level according to EGM1996 such that the Z position in the `map` frame is equivalent to meters above sea level. Whatever the choice is, the most important part is that the choice of reference position is clearly documented for users to avoid confusion.

When defining coordinate frames with respect to a global reference like the Earth:
- The default should be to align the X-axis east, Y-axis north, and the Z-axis up at the origin of the coordinate frame.
- If there is no other reference, the default position of the Z-axis should be zero at the height of the WGS84 ellipsoid.

In cases where there are application-specific requirements that cannot satisfy the above:
- Many as possible should still be met.
- For example, a robot starting up without an external reference device (such as GPS, compass, nor altimeter) can initialize the `map` at its current location with the Z-axis upward using an accelerometer.
- If the robot has a compass heading at startup, it can then also initialize X east, Y north.
- If the robot has an altimeter estimate at startup, it can initialize the height at Mean Sea Level (MSL).

The conventions above are strongly recommended for unstructured environments.

#### Map Conventions in Structured Environments

In structured environments, aligning the map with the environment may be more useful. An example of a structured environment is an office building interior, which is commonly rectilinear and has limited global localization methods. In such cases, aligning the map with the building is recommended, especially if the building layout is known a priori. Similarly, in an indoor environment, it is recommended to align the map at floor level. In cases where you are operating on multiple floors, it may make sense to have multiple coordinate frames, one for each floor.

If there is ambiguity, fall back to the conventions for unstructured environments above. Or if there is limited prior knowledge of the environment, the unstructured conventions can still be used in structured environments.

---

### Relationship between Frames

We have chosen a tree representation to attach all coordinate frames in a robot system to each other. Therefore, each coordinate frame has one parent coordinate frame and any number of child coordinate frames. The frames described in this REP are attached as follows:

```
map → odom → base_link → sensor frames
```

The `map` frame is the parent of `odom`, and `odom` is the parent of `base_link`. Although intuition might suggest that both `map` and `odom` should be attached directly to `base_link`, this is not allowed because each frame can only have one parent.

#### Extra Intermediate Frames

The graph shows the minimal representation of this structure. The basic topology should remain the same; however, it is acceptable to insert additional links in the graph to provide additional functionality.

**Example:**
- **Pressure Altitude Frame:** An additional coordinate frame can represent pressure altitude for flying vehicles. Pressure altitude is an approximation of altitude based on atmospheric barometric pressure. In flying applications, pressure altitude can be measured precisely using just a barometric altimeter. It may drift over time like odometry but will only drift vertically. To be useful, a `pressure_altitude` frame could be inserted between the inertially consistent `odom` frame and the `map` frame. An additional estimator would be required to estimate the offset of the `pressure_altitude` from the `map`, but this extra coordinate frame can support extra functionality without breaking the established abstraction.

**Example of Multi-Robot tf Graph Using ECEF:**

```
odom_1 → base_link1 → map_1 → earth
odom_2 → base_link2 → map_2 → earth
```

This is an example of a tf tree with two robots using different maps for localization and having a common frame `earth`.

The diagram above uses different `frame_ids` for clarity. However, for maximum reusability, it is recommended to use the canonical `frame_ids` on each robot and use a script to forward information off of the robot. When the information is forwarded, the `frame_ids` should be remapped to disambiguate which robot they are coming from and referencing.

---

### Frame Authorities

- **Transform from `odom` to `base_link`:** Computed and broadcast by one of the odometry sources.
  
- **Transform from `map` to `odom`:** Computed by a localization component. The localization component does not broadcast the transform from `map` to `base_link` directly. Instead, it first receives the transform from `odom` to `base_link` and uses this information to broadcast the transform from `map` to `odom`.
  
- **Transform from `earth` to `map`:** Statically published and configured based on the choice of the `map` frame. If not specifically configured, a fallback option is to use the initial position of the vehicle as the origin of the `map` frame. If the `map` is not georeferenced to support a simple static transform, the localization module can compute the estimated offset from the `map` to `odom` frame to publish the transform from `earth` to `map` frame.

In cases where the `map` frame's absolute position is unknown at startup, it can remain detached until the global position estimation can be adequately evaluated. This allows the robot to operate in the `odom` frame before localization in the `map` frame is initialized.

---

### Transitions Between Maps

When a robot travels a long distance, it is expected to transition between maps. In an outdoor context, the `map` coordinate frame is a Euclidean approximation of a vicinity; however, the Euclidean approximation breaks down at longer distances due to the Earth's curvature. In an indoor context, this can involve transitioning between two buildings with prior maps or navigating to a new floor in a building.

It is the responsibility of the localization frame authority to reparent the `odom` frame appropriately when moving between maps. The common implementation of computing the `map` to `odom` frame as the results of subtracting the `odom` to `base_link` from the localization fix (`map` to `base_link`) will take care of this implicitly when the choice of the `map` frame changes.

---

## Transformation Processes

Transforming coordinates from a global CRS (like WGS84) to a local frame involves a series of mathematical conversions. Two primary methods are commonly used in robotic navigation systems:

1. **WGS84 to ECEF to ENU**
2. **WGS84 to UTM**

Each method serves different purposes and offers unique advantages based on the application's requirements.

### 1. From WGS84 to ECEF to ENU

#### WGS84 to ECEF

The first step involves converting geographic coordinates (latitude, longitude, altitude) from the WGS84 system to the Earth-Centered, Earth-Fixed (ECEF) Cartesian coordinate system.

**Conversion Formulas:**
$$
X = (N + h) \cos(\phi) \cos(\lambda)
$$

$$
Y = (N + h) \cos(\phi) \sin(\lambda)
$$

$$
Z = \left[(1 - e^2) N + h\right] \sin(\phi)
$$

Where:
- $\phi$ = Latitude in radians
- $\lambda$ = Longitude in radians
- $h$ = Altitude in meters
- $e$ = First eccentricity of the ellipsoid
- $N$ = is the radius of curvature at latitude $\phi$ 
$$N = \frac{a}{\sqrt{1 - e^2 \sin^2(\phi)}}$$

**Parameters for WGS84:**
- **Semi-major axis (a)**: 6,378,137.0 meters
- **First eccentricity (e)**:
  \[
  e = \sqrt{1 - \left(\frac{b}{a}\right)^2} \approx 0.08181919084262149
  \]

#### ECEF to ENU

Once in ECEF coordinates, the next step is to convert to the local East-North-Up (ENU) coordinate system, which is more intuitive for navigation tasks.

**Steps:**

1. **Define a Reference Point:**
   - Typically the robot's starting position or a known landmark.
   - Coordinates of the reference point in ECEF: \((X_r, Y_r, Z_r)\)

2. **Calculate Relative Position:**
$$
\Delta X = X - X_r
$$

$$
\Delta Y = Y - Y_r
$$

$$
\Delta Z = Z - Z_r
$$


3. **Apply Rotation Matrix:**
$$
\begin{bmatrix}
E \\
N \\
U
\end{bmatrix}
=
\begin{bmatrix}
-\sin(\lambda_r) & \cos(\lambda_r) & 0 \\
-\sin(\phi_r) \cos(\lambda_r) & -\sin(\phi_r) \sin(\lambda_r) & \cos(\phi_r) \\
\cos(\phi_r) \cos(\lambda_r) & \cos(\phi_r) \sin(\lambda_r) & \sin(\phi_r)
\end{bmatrix}
\begin{bmatrix}
\Delta X \\
\Delta Y \\
\Delta Z
\end{bmatrix}
$$
  Where:
  - $\phi_r$ = Latitude of the reference point in radians
  - $\lambda_r$ = Longitude of the reference point in radians

**Result:**
- **E** (East), **N** (North), **U** (Up) coordinates in the local ENU frame.

### 2. From WGS84 to UTM

#### WGS84 to UTM

The UTM system projects geographic coordinates into a planar, Cartesian system, making it suitable for local and regional navigation.

**Conversion Steps:**

1. **Determine the UTM Zone:**

   $$
   \text{Zone} = \left\lfloor \frac{\text{Longitude} + 180}{6} \right\rfloor + 1
   $$

   - Longitude ranges from -180° to +180°
   - Zone numbers range from 1 to 60

2. **Apply the UTM Projection:**
   - Utilize projection libraries like **Proj4** to handle the complex mathematics of the transverse Mercator projection.
   - **Easting (E):** Distance from the central meridian of the UTM zone, offset by 500,000 meters to avoid negative values.
   - **Northing (N):** Distance from the equator. In the Southern Hemisphere, add 10,000,000 meters to ensure positive values.

3. **Handle Hemisphere:**
   - **Northern Hemisphere:** Northing starts at 0 meters at the equator.
   - **Southern Hemisphere:** Northing starts at 10,000,000 meters at the equator.

**Advantages:**
- Simplifies distance and area calculations in a local region.
- Provides a consistent, metric-based Cartesian system.


## Conclusion

Georeferencing is a fundamental process in robotic navigation systems, enabling the accurate translation of global positioning data into local coordinate frames that robots can utilize for precise movement and task execution. By understanding and implementing the transformation processes from WGS84 to ECEF and ENU or directly to UTM, and leveraging ROS 2's tf2 package for managing these transformations, robots can maintain accurate localization and navigation capabilities in diverse environments. This integration is essential for mitigating issues like odometry drift and ensuring reliable operation in both simulated and real-world scenarios.
