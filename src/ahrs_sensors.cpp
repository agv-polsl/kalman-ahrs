#include "ahrs/ahrs_sensors.hpp"

namespace ahrs {

void Sensor::calibrate_bias(const int readout_num) {
    sensor_readout read_sum;
    for (int i = 0; i < readout_num; i++) {
        auto readout = read();
        read_sum.x += readout.x;
        read_sum.y += readout.y;
        read_sum.z += readout.z;
    }

    bias = {read_sum.x / readout_num, read_sum.y / readout_num,
            read_sum.y / readout_num};
}

sensor_readout Sensor::fix_bias(sensor_readout readout) {
    return {readout.x - bias.x, readout.y - bias.y, readout.z - bias.z};
}

void Magnetometer::calibrate_bias(const int readout_num) {
    /* Before multiplicatice soft iron bias is calibrated, first the
     * hard iron additive bias must be set.
     */
    Sensor::calibrate_bias(readout_num);

    /* New readouts, fix the hard iron bias */
    sensor_readout read_sum;
    for (int i = 0; i < readout_num; i++) {
        auto readout = Sensor::fix_bias(read());
        read_sum.x += readout.x;
        read_sum.y += readout.y;
        read_sum.z += readout.z;
    }

    /* Calculate the soft iron bias */
    sensor_readout avg_biases = {read_sum.x / readout_num,
                                 read_sum.y / readout_num,
                                 read_sum.y / readout_num};

    double avg_bias = read_sum.x + read_sum.y + read_sum.z / 3;

    soft_iron_bias = {avg_bias / avg_biases.x, avg_bias / avg_biases.y,
                      avg_bias / avg_biases.z};
}

sensor_readout Magnetometer::fix_bias(sensor_readout readout) {
    readout = Sensor::fix_bias(readout);
    return {readout.x * soft_iron_bias.x, readout.y * soft_iron_bias.y,
            readout.z * soft_iron_bias.z};
}

}  // namespace ahrs
