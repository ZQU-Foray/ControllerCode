#include "AHRS.h"
#include "Libraries/Algorithm/alg_math/BasicMath.h"
#include <cmath>

namespace
{

constexpr float minNormSquared = 1.0e-12f;

bool IsFinite(float value)
{
    return std::isfinite(value);
}

} // namespace

namespace alg_estimate
{

bool AHRS::Init(const Config &config)
{
    Kp = 0.8f;
    Ki = 0.01f;
    Initialized = false;
    Reset();

    if (!IsFinite(config.Kp) || !IsFinite(config.Ki) || config.Kp < 0.0f || config.Ki < 0.0f)
    {
        return false;
    }

    Kp = config.Kp;
    Ki = config.Ki;
    Initialized = true;
    return true;
}

void AHRS::Reset()
{
    Q[0] = 1.0f;
    Q[1] = 0.0f;
    Q[2] = 0.0f;
    Q[3] = 0.0f;
    ResetIntegral();
    Yaw = 0.0f;
    Pitch = 0.0f;
    Roll = 0.0f;
    YawTotalAngle = 0.0f;
    YawAngleLast = 0.0f;
    YawRoundCount = 0;
    YawFirstFlag = true;
}

void AHRS::ResetIntegral()
{
    Integral[0] = 0.0f;
    Integral[1] = 0.0f;
    Integral[2] = 0.0f;
}

void AHRS::QuatDerivative(float q0,
                          float q1,
                          float q2,
                          float q3,
                          float gx,
                          float gy,
                          float gz,
                          float &dq0,
                          float &dq1,
                          float &dq2,
                          float &dq3)
{
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
}

bool AHRS::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    const bool inputFinite = IsFinite(gx) && IsFinite(gy) && IsFinite(gz) && IsFinite(ax) && IsFinite(ay) &&
                             IsFinite(az) && IsFinite(mx) && IsFinite(my) && IsFinite(mz) && IsFinite(dt);
    if (!Initialized || !inputFinite || dt <= 0.0f || dt >= 1.0f)
    {
        return false;
    }

    const float accelNormSquared = ax * ax + ay * ay + az * az;
    if (!IsFinite(accelNormSquared) || accelNormSquared <= minNormSquared)
    {
        return false;
    }

    const AHRS backup = *this;
    float q0 = Q[0];
    float q1 = Q[1];
    float q2 = Q[2];
    float q3 = Q[3];

    float norm = alg_math::InvSqrt(accelNormSquared);
    ax *= norm;
    ay *= norm;
    az *= norm;

    const float magneticNormSquared = mx * mx + my * my + mz * mz;
    if (IsFinite(magneticNormSquared) && magneticNormSquared > minNormSquared)
    {
        norm = alg_math::InvSqrt(magneticNormSquared);
        mx *= norm;
        my *= norm;
        mz *= norm;
    }
    else
    {
        mx = 0.0f;
        my = 0.0f;
        mz = 0.0f;
    }

    (void)mx;
    (void)my;
    (void)mz;

    const float q0q0 = q0 * q0;
    const float q0q1 = q0 * q1;
    const float q0q2 = q0 * q2;
    const float q1q1 = q1 * q1;
    const float q1q3 = q1 * q3;
    const float q2q2 = q2 * q2;
    const float q2q3 = q2 * q3;
    const float q3q3 = q3 * q3;

    const float vx = 2.0f * (q1q3 - q0q2);
    const float vy = 2.0f * (q0q1 + q2q3);
    const float vz = q0q0 - q1q1 - q2q2 + q3q3;

    const float ex = ay * vz - az * vy;
    const float ey = az * vx - ax * vz;
    const float ez = ax * vy - ay * vx;
    if (ex != 0.0f || ey != 0.0f || ez != 0.0f)
    {
        const float halfT = dt * 0.5f;
        Integral[0] += ex * Ki * halfT;
        Integral[1] += ey * Ki * halfT;
        Integral[2] += ez * Ki * halfT;

        gx += Kp * ex + Integral[0];
        gy += Kp * ey + Integral[1];
        gz += Kp * ez + Integral[2];
    }

    float k10, k11, k12, k13;
    QuatDerivative(q0, q1, q2, q3, gx, gy, gz, k10, k11, k12, k13);

    float k20, k21, k22, k23;
    QuatDerivative(q0 + 0.5f * dt * k10,
                   q1 + 0.5f * dt * k11,
                   q2 + 0.5f * dt * k12,
                   q3 + 0.5f * dt * k13,
                   gx,
                   gy,
                   gz,
                   k20,
                   k21,
                   k22,
                   k23);

    float k30, k31, k32, k33;
    QuatDerivative(q0 + 0.5f * dt * k20,
                   q1 + 0.5f * dt * k21,
                   q2 + 0.5f * dt * k22,
                   q3 + 0.5f * dt * k23,
                   gx,
                   gy,
                   gz,
                   k30,
                   k31,
                   k32,
                   k33);

    float k40, k41, k42, k43;
    QuatDerivative(q0 + dt * k30, q1 + dt * k31, q2 + dt * k32, q3 + dt * k33, gx, gy, gz, k40, k41, k42, k43);

    const float tempq0 = q0 + (dt / 6.0f) * (k10 + 2.0f * k20 + 2.0f * k30 + k40);
    const float tempq1 = q1 + (dt / 6.0f) * (k11 + 2.0f * k21 + 2.0f * k31 + k41);
    const float tempq2 = q2 + (dt / 6.0f) * (k12 + 2.0f * k22 + 2.0f * k32 + k42);
    const float tempq3 = q3 + (dt / 6.0f) * (k13 + 2.0f * k23 + 2.0f * k33 + k43);
    const float quaternionNormSquared = tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3;
    if (!IsFinite(quaternionNormSquared) || quaternionNormSquared <= minNormSquared)
    {
        *this = backup;
        return false;
    }

    norm = alg_math::InvSqrt(quaternionNormSquared);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

    const float yaw =
        std::atan2(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * alg_math::radToDeg;
    const float pitchInput = alg_math::Limit(-2.0f * q1 * q3 + 2.0f * q0 * q2, -1.0f, 1.0f);
    const float pitch = std::asin(pitchInput) * alg_math::radToDeg;
    const float roll =
        std::atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * alg_math::radToDeg;
    if (!IsFinite(q0) || !IsFinite(q1) || !IsFinite(q2) || !IsFinite(q3) || !IsFinite(yaw) || !IsFinite(pitch) ||
        !IsFinite(roll) || !IsFinite(Integral[0]) || !IsFinite(Integral[1]) || !IsFinite(Integral[2]))
    {
        *this = backup;
        return false;
    }

    Q[0] = q0;
    Q[1] = q1;
    Q[2] = q2;
    Q[3] = q3;
    Yaw = yaw;
    Pitch = pitch;
    Roll = roll;

    if (YawFirstFlag)
    {
        YawAngleLast = Yaw;
        YawTotalAngle = Yaw;
        YawFirstFlag = false;
    }
    else
    {
        if (Yaw - YawAngleLast > 180.0f)
        {
            YawRoundCount--;
        }
        else if (Yaw - YawAngleLast < -180.0f)
        {
            YawRoundCount++;
        }
        YawTotalAngle = YawRoundCount * 360.0f + Yaw;
        YawAngleLast = Yaw;
    }

    return true;
}

void AHRS::GetQuaternion(float q[4]) const
{
    q[0] = Q[0];
    q[1] = Q[1];
    q[2] = Q[2];
    q[3] = Q[3];
}

void AHRS::GetYawPitchRoll(float ypr[3]) const
{
    ypr[0] = Yaw;
    ypr[1] = Pitch;
    ypr[2] = Roll;
}

float AHRS::GetYawTotalAngle() const
{
    return YawTotalAngle;
}

bool AHRS::IsInitialized() const
{
    return Initialized;
}

} // namespace alg_estimate
