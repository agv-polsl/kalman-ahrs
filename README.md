![Build](https://github.com/agv-polsl/ekf-ahrs/workflows/Build/badge.svg)

# Kalman AHRS

Kalman filter based AHRS C++ library with sensor calibration and tilt
compensation built in.
Works with a gyroscope, accelerometer and magnetometer combo.
Built with no dependencies, utilises templating, deosn't rely on exceptions and
avoids dynamic memory allocation.
Suitable for Linux and real time embedded devices.
Tested on RaspberryPi with Pololu Minimu-9 v5.

# Build

Build follows standard CMake procedure; in project directory run:

```sh
mkdir build && cd build
cmake ..
cmake --build .
```

# Usage

The library is available via CMake build.
It doesn't feature a system wide installation, instead it is best to place the
source code inside your project and use it with `add_subdirectory(lib/minimu)`
and `target_link_libraries(target ahrs::ahrs)`.

To use the AHRS system implement the `Sensor` interface which requires the
`read` method.
This should be done for each of the AHRS sensors.
Init the system by injecting the `Sensor`s and passing initial sampling period.

```cpp
std::chrono::milliseconds dt = 5ms;
ahrs::Ahrs ahrs{gyro, acc, mag, dt};
```

Roll, pitch and yaw can be access by updating AHRS cycle with `update()`.
For example, this can be done in a thread loop:

```cpp
while (true) {
    std::this_thread::sleep_for(dt);
    auto readout = ahrs.update();
    std::cout << "x: " << readout.x << '\t' << "y: " << readout.y << '\t'
              << "z: " << readout.z << '\n';
}
```

Additionally a new sampling period can be passed to `update(new_dt)`.

## Calibration

Calibration of the IMU sensor is done by calling the `calibrate_imu()` method.
It should be done with the system lying still on even surface.

Calibration of the magnetometer/compass is done by calling `calibrate_mag()`.
During compass calibration the system should be turned around in every direction
as much as possible.

It is possible to access and init underlying Kalman filter covariance matrices.
Examine code header to view available methods.
Code follows standard Kalman filter naming scheme; for more information refer to
academic resources.
