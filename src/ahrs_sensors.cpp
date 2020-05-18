#include "ahrs/ahrs_sensors.h"

namespace ahrs {

sensor_readout ImuCalibratedSensor::read() {
    auto readout = imu_sensor.read();
    return {readout.x - offset_bias.x, readout.y - offset_bias.y,
            readout.z - offset_bias.z};
}

sensor_readout ImuCalibratedSensor::avg_n_readouts(int n) {
    sensor_readout read_sum = {0.0, 0.0, 0.0};
    for (int i = 0; i < n; i++) {
        auto readout = read();
        read_sum.x += readout.x;
        read_sum.y += readout.y;
        read_sum.z += readout.z;
    }

    return {read_sum.x / n, read_sum.y / n, read_sum.y / n};
}

void ImuCalibratedSensor::calibrate_bias(int num_of_samples) {
    offset_bias = avg_n_readouts(num_of_samples);
}

sensor_readout CompassCalibratedSensor::read() {
    auto readout = compass.read();

    readout = {readout.x - hard_iron_bias.x, readout.y - hard_iron_bias.y,
               readout.z - hard_iron_bias.z};
    readout = {readout.x * soft_iron_bias.x, readout.y * soft_iron_bias.y,
               readout.z * soft_iron_bias.z};

    return readout;
}

void CompassCalibratedSensor::calibrate_bias(int num_of_samples) {
    /* Mind the order of calibration, since the additive bias should be
     * calibrated before multiplicative one.
     */
    calibrate_hard_iron(num_of_samples);
    calibrate_soft_iron(num_of_samples);
}

sensor_readout update_min(sensor_readout readout, sensor_readout minr) {
    if (readout.x < minr.x) { minr.x = readout.x; }
    if (readout.y < minr.y) { minr.y = readout.y; }
    if (readout.z < minr.z) { minr.z = readout.z; }
    return minr;
}

sensor_readout update_max(sensor_readout readout, sensor_readout maxr) {
    if (readout.x > maxr.x) { maxr.x = readout.x; }
    if (readout.y > maxr.y) { maxr.y = readout.y; }
    if (readout.z > maxr.z) { maxr.z = readout.z; }
    return maxr;
}

void CompassCalibratedSensor::calibrate_hard_iron(int num_of_samples) {
    sensor_readout maxr = {0.0, 0.0, 0.0};
    sensor_readout minr = {0.0, 0.0, 0.0};
    for (int i = 0; i < num_of_samples; i++) {
        auto readout = read();
        maxr = update_max(readout, maxr);
        minr = update_min(readout, minr);
    }

    hard_iron_bias = { (maxr.x + minr.x) / 2, (maxr.y + minr.y) / 2 , (maxr.z + minr.z) / 2 };
}

void CompassCalibratedSensor::calibrate_soft_iron(int num_of_samples) {
    sensor_readout maxr = {0.0, 0.0, 0.0};
    sensor_readout minr = {0.0, 0.0, 0.0};
    for (int i = 0; i < num_of_samples; i++) {
        auto readout = read();
        maxr = update_max(readout, maxr);
        minr = update_min(readout, minr);
    }

    sensor_readout avg = { (maxr.x - minr.x) / 2, (maxr.y - minr.y) / 2 , (maxr.z - minr.z) / 2 };
    double avg_combined_axes = (avg.x + avg.y + avg.z / 3);
    soft_iron_bias = {avg_combined_axes / avg.x, avg_combined_axes / avg.y,
                      avg_combined_axes / avg.z};
}

}  // namespace ahrs
