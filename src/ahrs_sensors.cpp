#include "ahrs/ahrs_sensors.h"

namespace ahrs {

sensor_readout ImuCalibratedSensor::read() {
    auto readout = imu_sensor.read();
    return readout - offset_bias;
}

sensor_readout ImuCalibratedSensor::avg_n_readouts(int n) {
    sensor_readout read_sum = {0.0, 0.0, 0.0};
    for (int i = 0; i < n; i++) {
        auto readout = read();
        read_sum += readout;
    }
    return read_sum / n;
}

void ImuCalibratedSensor::calibrate_bias(int num_of_samples) {
    offset_bias = avg_n_readouts(num_of_samples);
}

sensor_readout CompassCalibratedSensor::read() {
    auto readout = compass.read();
    return (readout - hard_iron_bias) * soft_iron_bias;
}

void CompassCalibratedSensor::calibrate_bias(int num_of_samples) {
    /* Mind the order of calibration, since the additive bias should be
     * calibrated before multiplicative one.
     */
    calibrate_hard_iron(num_of_samples);
    calibrate_soft_iron(num_of_samples);
}

std::array<sensor_readout, 2>
CompassCalibratedSensor::find_minmax_in_each_dimension(int num_of_samples) {
    sensor_readout maxr = {0.0, 0.0, 0.0};
    sensor_readout minr = {0.0, 0.0, 0.0};

    for (int i = 0; i < num_of_samples; i++) {
        auto readout = read();
        maxr = update_max(readout, maxr);
        minr = update_min(readout, minr);
    }
    return {minr, maxr};
}

sensor_readout CompassCalibratedSensor::update_min(sensor_readout newr,
                                                   sensor_readout minr) {
    if (newr.x < minr.x) { minr.x = newr.x; }
    if (newr.y < minr.y) { minr.y = newr.y; }
    if (newr.z < minr.z) { minr.z = newr.z; }
    return minr;
}

sensor_readout CompassCalibratedSensor::update_max(sensor_readout newr,
                                                   sensor_readout maxr) {
    if (newr.x > maxr.x) { maxr.x = newr.x; }
    if (newr.y > maxr.y) { maxr.y = newr.y; }
    if (newr.z > maxr.z) { maxr.z = newr.z; }
    return maxr;
}

void CompassCalibratedSensor::calibrate_hard_iron(int num_of_samples) {
    auto [maxr, minr] = find_minmax_in_each_dimension(num_of_samples);
    hard_iron_bias = (maxr + minr) / 2;
}

void CompassCalibratedSensor::calibrate_soft_iron(int num_of_samples) {
    auto [maxr, minr] = find_minmax_in_each_dimension(num_of_samples);
    sensor_readout radius = (maxr - minr) / 2;
    double avg_radius = (radius.x + radius.y + radius.z / 3);
    soft_iron_bias = {avg_radius / radius.x, avg_radius / radius.y,
                      avg_radius / radius.z};
}

}  // namespace ahrs
