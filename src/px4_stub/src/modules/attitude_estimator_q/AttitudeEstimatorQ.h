#pragma once

#include <matrix/math.hpp>

struct AttitudeEstimatorQParams {
    float w_acc = 0.2f;            // ATT_W_ACC
    float w_mag = 0.1f;            // ATT_W_MAG
    float w_ext_hdg = 0.1f;        // ATT_W_EXT_HDG
    float w_gyro_bias = 0.1f;      // ATT_W_GYRO_BIAS
    float mag_decl = 0.0f;         // ATT_MAG_DECL (rad)
    int ext_hdg_m = 0;             // ATT_EXT_HDG_M (0: mag, 1: vision, 2: mocap)
    int acc_comp = 1;              // ATT_ACC_COMP
    float bias_max = 0.05f;        // Maximum gyro bias (rad/s)
    int has_mag = 1;               // SYS_HAS_MAG
};

class AttitudeEstimatorQ {
public:
    AttitudeEstimatorQ() {
        _q = matrix::Quatf();
    }

    void setParams(const AttitudeEstimatorQParams& params) {
        _params = params;
        if (!_params.has_mag) {
            _params.w_mag = 0.0f;
        }
        if (_params.w_mag < 1e-6f) {
            _mag(0) = 1.0f;
            _mag(1) = 0.0f;
            _mag(2) = 0.0f;
        }
        update_mag_declination(_params.mag_decl);
    }

    bool update(float dt, 
                const matrix::Vector3f& accel, 
                const matrix::Vector3f& gyro,
                const matrix::Vector3f& mag,
                const matrix::Vector3f& pos_acc = matrix::Vector3f());

    matrix::Quatf getQuat() const { return _q; }
    matrix::Vector3f getRates() const { return _rates; }
    matrix::Vector3f getGyroBias() const { return _gyro_bias; }

    // Manual initialization if needed
    bool init(const matrix::Vector3f& accel, const matrix::Vector3f& mag);

private:
    void update_mag_declination(float new_declination);

    AttitudeEstimatorQParams _params{};

    matrix::Vector3f _gyro_bias{};
    matrix::Vector3f _rates{};
    matrix::Quatf _q{};
    matrix::Vector3f _mag{}; // stored inside class for init

    float _mag_decl{0.0f};
    bool _initialized{false};
    bool _ext_hdg_good{false};
};
