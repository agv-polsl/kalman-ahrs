#include "ahrs/sensors.h"

namespace ahrs {

static sensor_readout update_min(sensor_readout newr,
                                 sensor_readout minr) noexcept {
    if (newr.x < minr.x) { minr.x = newr.x; }
    if (newr.y < minr.y) { minr.y = newr.y; }
    if (newr.z < minr.z) { minr.z = newr.z; }
    return minr;
}

static sensor_readout update_max(sensor_readout newr,
                                 sensor_readout maxr) noexcept {
    if (newr.x > maxr.x) { maxr.x = newr.x; }
    if (newr.y > maxr.y) { maxr.y = newr.y; }
    if (newr.z > maxr.z) { maxr.z = newr.z; }
    return maxr;
}

sensor_readout ImuCalibratedSensor::read() const {
    auto readout = imu_sensor.read();
    return readout - offset_bias;
}

sensor_readout ImuCalibratedSensor::avg_n_readouts(const size_t n) const {
    sensor_readout read_sum = {0.0, 0.0, 0.0};
    for (size_t i = 0; i < n; i++) {
        auto readout = read();
        read_sum += readout;
    }
    return read_sum / static_cast<double>(n);
}

void ImuCalibratedSensor::calibrate_bias(const size_t num_of_samples) {
    offset_bias = {0.0, 0.0, 0.0};
    auto avg = avg_n_readouts(num_of_samples);
    offset_bias = avg;
}

void AccelCalibratedSensor::calibrate_bias(const size_t num_of_samples) {
    ImuCalibratedSensor::calibrate_bias(num_of_samples);
    offset_bias.z = 0;
}

sensor_readout CompassCalibratedSensor::read() const {
    auto readout = compass.read();
    return (readout - hard_iron_bias) * soft_iron_bias;
}

void CompassCalibratedSensor::calibrate_bias(
    const size_t num_of_samples) {
    /* Mind the order of calibration, since the additive bias should be
     * calibrated before multiplicative one.
     */
    calibrate_hard_iron(num_of_samples);
    calibrate_soft_iron(num_of_samples);
}

std::array<sensor_readout, 2>
CompassCalibratedSensor::find_minmax_in_each_dimension(
    const size_t num_of_samples) const {
    sensor_readout maxr = {0.0, 0.0, 0.0};
    sensor_readout minr = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < num_of_samples; i++) {
        auto readout = read();
        maxr = update_max(readout, maxr);
        minr = update_min(readout, minr);
    }
    return {minr, maxr};
}

void CompassCalibratedSensor::calibrate_hard_iron(
    const size_t num_of_samples) {
    hard_iron_bias = {0.0, 0.0, 0.0};
    auto [minr, maxr] = find_minmax_in_each_dimension(num_of_samples);
    hard_iron_bias = (maxr + minr) / 2;
}

void CompassCalibratedSensor::calibrate_soft_iron(
    const size_t num_of_samples) {
    soft_iron_bias = {1.0, 1.0, 1.0};
    auto [minr, maxr] = find_minmax_in_each_dimension(num_of_samples);
    sensor_readout radius = (maxr - minr) / 2;
    double avg_radius = (radius.x + radius.y + radius.z) / 3;
    soft_iron_bias = {avg_radius / radius.x,
                      avg_radius / radius.y,
                      avg_radius / radius.z};
}

}  // namespace ahrs
