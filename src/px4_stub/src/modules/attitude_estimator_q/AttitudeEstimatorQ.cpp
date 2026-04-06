#include "AttitudeEstimatorQ.h"
#include <cmath>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;

static constexpr float CONSTANTS_ONE_G = 9.80665f;

bool AttitudeEstimatorQ::init(const Vector3f& accel, const Vector3f& mag) {
    if (accel.length() < 0.01f) return false;

    _mag = mag;
    if (_mag.length() < 0.01f) {
        _mag(0) = 1.0f;
        _mag(1) = 0.0f;
        _mag(2) = 0.0f;
    }

    // 'k' is Earth Z axis (Down) unit vector in body frame
    Vector3f k = -accel;
    k.normalize();

    // 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
    Vector3f i = (_mag - k * (_mag * k));
    i.normalize();

    // 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
    Vector3f j = k % i;

    // Fill rotation matrix
    Dcmf R;
    R.row(0) = i;
    R.row(1) = j;
    R.row(2) = k;

    // Convert to quaternion
    _q = R;

    // Compensate for magnetic declination
    Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
    _q = _q * decl_rotation;

    _q.normalize();

    if (_q.isAllFinite() && _q.length() > 0.95f && _q.length() < 1.05f) {
        _initialized = true;
    } else {
        _initialized = false;
    }
    return _initialized;
}

bool AttitudeEstimatorQ::update(float dt,
                                const Vector3f& accel,
                                const Vector3f& gyro,
                                const Vector3f& mag,
                                const Vector3f& pos_acc)
{
    if (!_initialized) {
        return init(accel, mag);
    }

    Quatf q_last = _q;

    // Angular rate of correction
    Vector3f corr;
    float spinRate = gyro.length();

    // Magnetometer correction
    if (_params.ext_hdg_m == 0 || !_ext_hdg_good) {
        // Project mag field vector to global frame and extract XY component
        Vector3f m = mag;
        if (m.length() < 0.01f) {
            m(0) = 1.0f; m(1) = 0.0f; m(2) = 0.0f;
        }

        Vector3f mag_earth = _q.rotateVector(m);
        float mag_err = wrap_pi(std::atan2(mag_earth(1), mag_earth(0)) - _mag_decl);
        float gainMult = 1.0f;
        const float fifty_dps = 0.873f;

        if (spinRate > fifty_dps) {
            gainMult = std::fmin(spinRate / fifty_dps, 10.0f);
        }

        // Project magnetometer correction to body frame
        corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -mag_err)) * _params.w_mag * gainMult;
    }

    _q.normalize();

    // Accelerometer correction
    // Optimized version with dropped zeros
    Vector3f k(
        2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
        2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
        (_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
    );

    const float accel_norm_sq = accel.norm_squared();
    const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
    const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;

    if (_params.acc_comp || ((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
                             (accel_norm_sq < upper_accel_limit * upper_accel_limit))) {

        corr += (k % (accel - pos_acc).normalized()) * _params.w_acc;
    }

    // Gyro bias estimation
    if (spinRate < 0.175f) {
        _gyro_bias += corr * (_params.w_gyro_bias * dt);

        for (int idx = 0; idx < 3; idx++) {
            _gyro_bias(idx) = std::fmax(std::fmin(_gyro_bias(idx), _params.bias_max), -_params.bias_max);
        }
    }

    _rates = gyro + _gyro_bias;

    // Feed forward gyro
    corr += _rates;

    // Apply correction to state
    _q += _q.derivative1(corr) * dt;

    // Normalize quaternion
    _q.normalize();

    if (!_q.isAllFinite()) {
        // Reset quaternion to last good state
        _q = q_last;
        _rates.zero();
        _gyro_bias.zero();
        return false;
    }

    return true;
}

void AttitudeEstimatorQ::update_mag_declination(float new_declination)
{
    if (!_initialized || std::fabs(new_declination - _mag_decl) < 0.0001f) {
        _mag_decl = new_declination;
    } else {
        Quatf decl_rotation = Eulerf(0.0f, 0.0f, new_declination - _mag_decl);
        _q = _q * decl_rotation;
        _mag_decl = new_declination;
    }
}
