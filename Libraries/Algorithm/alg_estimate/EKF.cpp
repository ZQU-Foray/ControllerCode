#include "EKF.h"
#include <algorithm>
#include <cmath>

namespace alg_estimate
{
namespace
{

constexpr float gravityAcceleration = 9.7883f;
constexpr float chiSquareThreshold = 1.0e-3f;
constexpr float accelGateG = 0.10f;
constexpr float gyroDynamicRadS = 0.8f;
constexpr float rDynamicScale = 9000.0f;
constexpr float rGyroScale = 120.0f;
constexpr float rMax = 1000000.0f;
constexpr float radToDeg = 57.295779513f;
constexpr float piHalf = 1.5707963f;

// 初始F
constexpr float f0[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
// 初始P
constexpr float p0[36] = {100000, 0.1, 0.1,    0.1, 0.1, 0.1, 0.1, 100000, 0.1, 0.1,    0.1, 0.1,
                          0.1,    0.1, 100000, 0.1, 0.1, 0.1, 0.1, 0.1,    0.1, 100000, 0.1, 0.1,
                          0.1,    0.1, 0.1,    0.1, 100, 0.1, 0.1, 0.1,    0.1, 0.1,    0.1, 100};

float InvSqrt(float x)
{
    if (!std::isfinite(x) || x <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / std::sqrt(x);
}

} // namespace

bool EKF::Init(const float initQuaternion[4], const Config &config)
{
    Initialized = false;
    LastStatus = ARM_MATH_ARGUMENT_ERROR;
    Q1 = 0.0f;
    Q2 = 0.0f;
    R = 0.0f;
    Lambda = 1.0f;
    AccLpfCoef = 0.0f;
    ExternalRBoost = 1.0f;
    ChiSquareThreshold = chiSquareThreshold;
    ConvergeFlag = false;
    StableFlag = false;
    ErrorCount = 0;
    UpdateCount = 0;
    FreezeUpdate = false;
    YawRoundCount = 0;
    YawAngleLast = 0.0f;
    Yaw = 0.0f;
    Pitch = 0.0f;
    Roll = 0.0f;
    YawTotalAngle = 0.0f;
    GyroBias[0] = 0.0f;
    GyroBias[1] = 0.0f;
    GyroBias[2] = 0.0f;
    Gyro[0] = 0.0f;
    Gyro[1] = 0.0f;
    Gyro[2] = 0.0f;
    Accel[0] = 0.0f;
    Accel[1] = 0.0f;
    Accel[2] = 0.0f;

    if (initQuaternion == nullptr || !std::isfinite(config.ProcessNoise1) || !std::isfinite(config.ProcessNoise2) ||
        !std::isfinite(config.MeasureNoise) || !std::isfinite(config.Lambda) || !std::isfinite(config.AccLpfCoef) ||
        config.ProcessNoise1 < 0.0f || config.ProcessNoise2 < 0.0f || config.MeasureNoise <= 0.0f ||
        config.Lambda <= 0.0f || config.Lambda > 1.0f || config.AccLpfCoef < 0.0f)
    {
        return false;
    }

    float quaternionNormSquared = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!std::isfinite(initQuaternion[i]))
        {
            return false;
        }
        quaternionNormSquared += initQuaternion[i] * initQuaternion[i];
    }
    const float quaternionInvNorm = InvSqrt(quaternionNormSquared);
    if (quaternionInvNorm <= 0.0f)
    {
        return false;
    }

    Q1 = config.ProcessNoise1;
    Q2 = config.ProcessNoise2;
    R = config.MeasureNoise;
    Lambda = config.Lambda;
    AccLpfCoef = config.AccLpfCoef;

    if (!Kf.Init(6, 0, 3))
    {
        LastStatus = Kf.GetLastStatus();
        return false;
    }
    Kf.UserData = this;
    arm_mat_init_f32(&ChiSquare, 1, 1, ChiSquareData);

    for (uint8_t i = 0; i < 4; i++)
    {
        Kf.XhatData[i] = initQuaternion[i] * quaternionInvNorm;
        Q[i] = Kf.XhatData[i];
    }

    // F、P初值
    std::copy(f0, f0 + 36, Kf.FData.begin());
    std::copy(p0, p0 + 36, Kf.PData.begin());

    // 用自定义回调替换标准KF环节
    Kf.SkipEq1 = true;
    Kf.SkipEq3 = true;
    Kf.SkipEq4 = true;
    Kf.UserFunc0 = &EKF::UserFunc0;
    Kf.UserFunc1 = &EKF::UserFunc1;
    Kf.UserFunc2 = &EKF::UserFunc2;
    Kf.UserFunc3 = &EKF::UserFunc3;
    Initialized = true;
    LastStatus = ARM_MATH_SUCCESS;
    return true;
}

bool EKF::Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    const bool inputFinite = std::isfinite(gx) && std::isfinite(gy) && std::isfinite(gz) && std::isfinite(ax) &&
                             std::isfinite(ay) && std::isfinite(az) && std::isfinite(dt);
    const float rawAccelNormSquared = ax * ax + ay * ay + az * az;
    if (!Initialized || !inputFinite || dt <= 0.0f || dt >= 1.0f || !std::isfinite(rawAccelNormSquared) ||
        rawAccelNormSquared < 1.0e-8f)
    {
        LastStatus = ARM_MATH_ARGUMENT_ERROR;
        return false;
    }

    const float oldDt = Dt;
    const float oldGyro[3] = {Gyro[0], Gyro[1], Gyro[2]};
    const float oldAccel[3] = {Accel[0], Accel[1], Accel[2]};
    const bool oldStableFlag = StableFlag;
    const bool oldConvergeFlag = ConvergeFlag;
    const uint64_t oldErrorCount = ErrorCount;
    const float oldAdaptiveGainScale = AdaptiveGainScale;

    Dt = dt;

    Gyro[0] = gx - GyroBias[0];
    Gyro[1] = gy - GyroBias[1];
    Gyro[2] = gz - GyroBias[2];

    float halfgxdt = 0.5f * Gyro[0] * dt;
    float halfgydt = 0.5f * Gyro[1] * dt;
    float halfgzdt = 0.5f * Gyro[2] * dt;

    // F重置为单位阵后填入陀螺项
    std::copy(f0, f0 + 36, Kf.FData.begin());
    Kf.FData[1] = -halfgxdt;
    Kf.FData[2] = -halfgydt;
    Kf.FData[3] = -halfgzdt;
    Kf.FData[6] = halfgxdt;
    Kf.FData[8] = halfgzdt;
    Kf.FData[9] = -halfgydt;
    Kf.FData[12] = halfgydt;
    Kf.FData[13] = -halfgzdt;
    Kf.FData[15] = halfgxdt;
    Kf.FData[18] = halfgzdt;
    Kf.FData[19] = halfgydt;
    Kf.FData[20] = -halfgxdt;

    if (UpdateCount == 0)
    {
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;
    }
    if (AccLpfCoef > 0.0f)
    {
        Accel[0] = Accel[0] * AccLpfCoef / (Dt + AccLpfCoef) + ax * Dt / (Dt + AccLpfCoef);
        Accel[1] = Accel[1] * AccLpfCoef / (Dt + AccLpfCoef) + ay * Dt / (Dt + AccLpfCoef);
        Accel[2] = Accel[2] * AccLpfCoef / (Dt + AccLpfCoef) + az * Dt / (Dt + AccLpfCoef);
    }
    else
    {
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;
    }

    float accelNormSq = Accel[0] * Accel[0] + Accel[1] * Accel[1] + Accel[2] * Accel[2];
    if (!std::isfinite(accelNormSq) || accelNormSq < 1.0e-8f)
    {
        Dt = oldDt;
        std::copy(oldGyro, oldGyro + 3, Gyro);
        std::copy(oldAccel, oldAccel + 3, Accel);
        LastStatus = ARM_MATH_NANINF;
        return false;
    }
    float accelInvNorm = InvSqrt(accelNormSq);
    Kf.MeasuredVector[0] = Accel[0] * accelInvNorm;
    Kf.MeasuredVector[1] = Accel[1] * accelInvNorm;
    Kf.MeasuredVector[2] = Accel[2] * accelInvNorm;

    GyroNorm = std::sqrt(Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2]);
    AccelNorm = 1.0f / accelInvNorm;

    // 静态稳定标志
    if (GyroNorm < 0.3f && std::fabs(AccelNorm - gravityAcceleration) < 0.35f)
    {
        StableFlag = true;
    }
    else
    {
        StableFlag = false;
    }

    // 自适应量测噪声（加速度门限 + 角速度门限）
    float accelGError = std::fabs(AccelNorm - gravityAcceleration) / gravityAcceleration;
    float accelGate = accelGError / accelGateG;
    if (accelGate > 1.0f)
    {
        accelGate = 1.0f;
    }
    float gyroGate = (GyroNorm - gyroDynamicRadS) / gyroDynamicRadS;
    if (gyroGate < 0.0f)
    {
        gyroGate = 0.0f;
    }
    else if (gyroGate > 1.0f)
    {
        gyroGate = 1.0f;
    }
    float dynamicR = R * (1.0f + rDynamicScale * accelGate * accelGate + rGyroScale * gyroGate * gyroGate);
    dynamicR *= ExternalRBoost;
    if (!std::isfinite(dynamicR) || dynamicR > rMax)
    {
        dynamicR = rMax;
    }

    // Q、R 填充
    Kf.QData[0] = Q1 * Dt;
    Kf.QData[7] = Q1 * Dt;
    Kf.QData[14] = Q1 * Dt;
    Kf.QData[21] = Q1 * Dt;
    Kf.QData[28] = Q2 * Dt;
    Kf.QData[35] = Q2 * Dt;
    Kf.RData[0] = dynamicR;
    Kf.RData[4] = dynamicR;
    Kf.RData[8] = dynamicR;

    if (!Kf.Update())
    {
        Dt = oldDt;
        std::copy(oldGyro, oldGyro + 3, Gyro);
        std::copy(oldAccel, oldAccel + 3, Accel);
        StableFlag = oldStableFlag;
        ConvergeFlag = oldConvergeFlag;
        ErrorCount = oldErrorCount;
        AdaptiveGainScale = oldAdaptiveGainScale;
        LastStatus = Kf.GetLastStatus();
        return false;
    }

    const float *filteredValue = Kf.GetFilteredValue();
    if (filteredValue == nullptr)
    {
        LastStatus = ARM_MATH_ARGUMENT_ERROR;
        return false;
    }

    const float q0 = filteredValue[0];
    const float q1 = filteredValue[1];
    const float q2 = filteredValue[2];
    const float q3 = filteredValue[3];
    const float yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 2.0f * (q0 * q0 + q1 * q1) - 1.0f) * radToDeg;
    const float pitch = std::atan2(2.0f * (q0 * q1 + q2 * q3), 2.0f * (q0 * q0 + q3 * q3) - 1.0f) * radToDeg;
    float rollInput = -2.0f * (q1 * q3 - q0 * q2);
    if (rollInput > 1.0f)
    {
        rollInput = 1.0f;
    }
    else if (rollInput < -1.0f)
    {
        rollInput = -1.0f;
    }
    const float roll = std::asin(rollInput) * radToDeg;
    if (!std::isfinite(q0) || !std::isfinite(q1) || !std::isfinite(q2) || !std::isfinite(q3) ||
        !std::isfinite(filteredValue[4]) || !std::isfinite(filteredValue[5]) || !std::isfinite(yaw) ||
        !std::isfinite(pitch) || !std::isfinite(roll))
    {
        LastStatus = ARM_MATH_NANINF;
        return false;
    }

    Q[0] = q0;
    Q[1] = q1;
    Q[2] = q2;
    Q[3] = q3;
    GyroBias[0] = filteredValue[4];
    GyroBias[1] = filteredValue[5];
    GyroBias[2] = 0.0f;
    Yaw = yaw;
    Pitch = pitch;
    Roll = roll;

    // 偏航角跨 ±180 计数，累计总角度
    if (Yaw - YawAngleLast > 180.0f)
    {
        YawRoundCount--;
    }
    else if (Yaw - YawAngleLast < -180.0f)
    {
        YawRoundCount++;
    }
    YawTotalAngle = 360.0f * YawRoundCount + Yaw;
    YawAngleLast = Yaw;
    UpdateCount++;
    LastStatus = ARM_MATH_SUCCESS;
    return true;
}

arm_status EKF::UserFunc0(KalmanFilter &kf)
{
    if (kf.UserData == nullptr)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    return static_cast<EKF *>(kf.UserData)->Observe();
}

arm_status EKF::UserFunc1(KalmanFilter &kf)
{
    if (kf.UserData == nullptr)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    return static_cast<EKF *>(kf.UserData)->PredictAndLinearize();
}

arm_status EKF::UserFunc2(KalmanFilter &kf)
{
    if (kf.UserData == nullptr)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    return static_cast<EKF *>(kf.UserData)->SetH();
}

arm_status EKF::UserFunc3(KalmanFilter &kf)
{
    if (kf.UserData == nullptr)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    return static_cast<EKF *>(kf.UserData)->XhatUpdate();
}

/**
 * @brief 对应 UserFunc0（Observe）：保存 P/K/H 快照（调试用）
 */
arm_status EKF::Observe()
{
    std::copy(Kf.PData.begin(), Kf.PData.end(), PSnap);
    std::copy(Kf.KData.begin(), Kf.KData.end(), KSnap);
    std::copy(Kf.HData.begin(), Kf.HData.end(), HSnap);
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 对应 UserFunc1：四元数精确预测 + 归一化 + F 线性化 + P 渐消
 */
arm_status EKF::PredictAndLinearize()
{
    arm_status status = PredictQuaternionExact();
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    float q0 = Kf.XhatminusData[0];
    float q1 = Kf.XhatminusData[1];
    float q2 = Kf.XhatminusData[2];
    float q3 = Kf.XhatminusData[3];

    float qInvNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qInvNorm <= 0.0f)
    {
        return ARM_MATH_NANINF;
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        Kf.XhatminusData[i] *= qInvNorm;
    }

    // F 的零偏相关项线性化
    Kf.FData[4] = q1 * Dt / 2.0f;
    Kf.FData[5] = q2 * Dt / 2.0f;
    Kf.FData[10] = -q0 * Dt / 2.0f;
    Kf.FData[11] = q3 * Dt / 2.0f;
    Kf.FData[16] = -q3 * Dt / 2.0f;
    Kf.FData[17] = -q0 * Dt / 2.0f;
    Kf.FData[22] = q2 * Dt / 2.0f;
    Kf.FData[23] = -q1 * Dt / 2.0f;

    // P 渐消（仅零偏对角线）
    Kf.PData[28] /= Lambda;
    Kf.PData[35] /= Lambda;
    if (Kf.PData[28] > 10000.0f)
    {
        Kf.PData[28] = 10000.0f;
    }
    if (Kf.PData[35] > 10000.0f)
    {
        Kf.PData[35] = 10000.0f;
    }
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 四元数精确预测（指数积分，对应 PredictQuaternionExact）
 */
arm_status EKF::PredictQuaternionExact()
{
    float q0 = Kf.XhatData[0];
    float q1 = Kf.XhatData[1];
    float q2 = Kf.XhatData[2];
    float q3 = Kf.XhatData[3];
    float wx = Gyro[0];
    float wy = Gyro[1];
    float wz = Gyro[2];
    float gyroNorm = std::sqrt(wx * wx + wy * wy + wz * wz);
    float halfAngle = 0.5f * gyroNorm * Dt;
    float c;
    float scale;

    if (!std::isfinite(gyroNorm) || gyroNorm < 1.0e-8f || !std::isfinite(halfAngle))
    {
        c = 1.0f;
        scale = 0.5f * Dt;
    }
    else
    {
        c = std::cos(halfAngle);
        scale = std::sin(halfAngle) / gyroNorm;
    }

    float sx = wx * scale;
    float sy = wy * scale;
    float sz = wz * scale;

    Kf.XhatminusData[0] = q0 * c - q1 * sx - q2 * sy - q3 * sz;
    Kf.XhatminusData[1] = q0 * sx + q1 * c + q2 * sz - q3 * sy;
    Kf.XhatminusData[2] = q0 * sy - q1 * sz + q2 * c + q3 * sx;
    Kf.XhatminusData[3] = q0 * sz + q1 * sy - q2 * sx + q3 * c;
    Kf.XhatminusData[4] = Kf.XhatData[4];
    Kf.XhatminusData[5] = Kf.XhatData[5];

    for (uint8_t i = 0; i < 6; i++)
    {
        if (!std::isfinite(Kf.XhatminusData[i]))
        {
            return ARM_MATH_NANINF;
        }
    }
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 对应 UserFunc2（SetH）：观测雅可比（3x6，仅四元数列非零）
 */
arm_status EKF::SetH()
{
    float dq0 = 2.0f * Kf.XhatminusData[0];
    float dq1 = 2.0f * Kf.XhatminusData[1];
    float dq2 = 2.0f * Kf.XhatminusData[2];
    float dq3 = 2.0f * Kf.XhatminusData[3];

    std::fill(Kf.HData.begin(), Kf.HData.end(), 0.0f);

    Kf.HData[0] = -dq2;
    Kf.HData[1] = dq3;
    Kf.HData[2] = -dq0;
    Kf.HData[3] = dq1;
    Kf.HData[6] = dq1;
    Kf.HData[7] = dq0;
    Kf.HData[8] = dq3;
    Kf.HData[9] = dq2;
    Kf.HData[12] = dq0;
    Kf.HData[13] = -dq1;
    Kf.HData[14] = -dq2;
    Kf.HData[15] = dq3;
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 对应 UserFunc3：卡方检验 + 自适应增益 + 增益计算 + 后验状态
 */
arm_status EKF::XhatUpdate()
{
    KalmanFilter &kf = Kf;

    if (FreezeUpdate)
    {
        // 冻结：仅预测，后验 = 先验
        std::copy(kf.XhatminusData.begin(), kf.XhatminusData.end(), kf.XhatData.begin());
        std::copy(kf.PminusData.begin(), kf.PminusData.end(), kf.PData.begin());
        kf.SkipEq5 = true;
        return ARM_MATH_SUCCESS;
    }

    // S = H·P⁻·Hᵀ + R，inv(S) → TempMatrix1
    arm_status status = arm_mat_trans_f32(&kf.H, &kf.Ht);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    kf.TempMatrix.numRows = kf.H.numRows;
    kf.TempMatrix.numCols = kf.Pminus.numCols;
    status = arm_mat_mult_f32(&kf.H, &kf.Pminus, &kf.TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    kf.TempMatrix1.numRows = kf.TempMatrix.numRows;
    kf.TempMatrix1.numCols = kf.Ht.numCols;
    status = arm_mat_mult_f32(&kf.TempMatrix, &kf.Ht, &kf.TempMatrix1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    kf.S.numRows = kf.R.numRows;
    kf.S.numCols = kf.R.numCols;
    status = arm_mat_add_f32(&kf.TempMatrix1, &kf.R, &kf.S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    status = arm_mat_inverse_f32(&kf.S, &kf.TempMatrix1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    // 预测重力方向（由先验四元数）
    const float q0 = kf.XhatminusData[0];
    const float q1 = kf.XhatminusData[1];
    const float q2 = kf.XhatminusData[2];
    const float q3 = kf.XhatminusData[3];
    kf.TempVectorData[0] = 2.0f * (q1 * q3 - q0 * q2);
    kf.TempVectorData[1] = 2.0f * (q0 * q1 + q2 * q3);
    kf.TempVectorData[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (uint8_t i = 0; i < 3; i++)
    {
        float cosine = std::fabs(kf.TempVectorData[i]);
        if (cosine > 1.0f)
        {
            cosine = 1.0f;
        }
        OrientationCosine[i] = std::acos(cosine);
    }

    // 新息
    kf.TempVector1.numRows = kf.Z.numRows;
    kf.TempVector1.numCols = 1;
    kf.TempVectorData1[0] = kf.ZData[0] - kf.TempVectorData[0];
    kf.TempVectorData1[1] = kf.ZData[1] - kf.TempVectorData[1];
    kf.TempVectorData1[2] = kf.ZData[2] - kf.TempVectorData[2];

    // 卡方检验：χ² = (Z-h)ᵀ·S⁻¹·(Z-h)
    kf.TempMatrix.numRows = kf.TempVector1.numRows;
    kf.TempMatrix.numCols = 1;
    status = arm_mat_mult_f32(&kf.TempMatrix1, &kf.TempVector1, &kf.TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    kf.TempVector.numRows = 1;
    kf.TempVector.numCols = kf.TempVector1.numRows;
    status = arm_mat_trans_f32(&kf.TempVector1, &kf.TempVector);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    status = arm_mat_mult_f32(&kf.TempVector, &kf.TempMatrix, &ChiSquare);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    if (!std::isfinite(ChiSquareData[0]))
    {
        return ARM_MATH_NANINF;
    }

    if (ChiSquareData[0] < 0.5f * ChiSquareThreshold)
    {
        ConvergeFlag = true;
    }

    if (ChiSquareData[0] > ChiSquareThreshold && ConvergeFlag)
    {
        // 持续异常：累积错误计数
        if (StableFlag)
        {
            ErrorCount++;
        }
        else
        {
            ErrorCount = 0;
        }

        if (ErrorCount > 50)
        {
            // 判定发散，重置收敛标志，允许正常更新
            ConvergeFlag = false;
            kf.SkipEq5 = false;
        }
        else
        {
            // 短暂异常：保持先验，不更新
            std::copy(kf.XhatminusData.begin(), kf.XhatminusData.end(), kf.XhatData.begin());
            std::copy(kf.PminusData.begin(), kf.PminusData.end(), kf.PData.begin());
            kf.SkipEq5 = true;
            return ARM_MATH_SUCCESS;
        }
    }
    else
    {
        // 正常：计算自适应增益
        if (ChiSquareData[0] > 0.1f * ChiSquareThreshold && ConvergeFlag)
        {
            AdaptiveGainScale = (ChiSquareThreshold - ChiSquareData[0]) / (0.9f * ChiSquareThreshold);
        }
        else
        {
            AdaptiveGainScale = 1.0f;
        }
        ErrorCount = 0;
        kf.SkipEq5 = false;
    }

    // K = P⁻·Hᵀ·S⁻¹
    kf.TempMatrix.numRows = kf.Pminus.numRows;
    kf.TempMatrix.numCols = kf.Ht.numCols;
    status = arm_mat_mult_f32(&kf.Pminus, &kf.Ht, &kf.TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    status = arm_mat_mult_f32(&kf.TempMatrix, &kf.TempMatrix1, &kf.K);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    // 自适应增益缩放
    for (uint8_t i = 0; i < kf.K.numRows * kf.K.numCols; i++)
    {
        kf.KData[i] *= AdaptiveGainScale;
    }
    // 零偏行的增益按姿态余弦缩放
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf.KData[i * 3 + j] *= OrientationCosine[i - 4] / piHalf;
        }
    }

    // 修正量 = K·新息（6x1）
    kf.TempVector.numRows = kf.K.numRows;
    kf.TempVector.numCols = 1;
    status = arm_mat_mult_f32(&kf.K, &kf.TempVector1, &kf.TempVector);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    // 零漂修正限幅
    if (ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf.TempVectorData[i] > 1.0e-2f * Dt)
            {
                kf.TempVectorData[i] = 1.0e-2f * Dt;
            }
            if (kf.TempVectorData[i] < -1.0e-2f * Dt)
            {
                kf.TempVectorData[i] = -1.0e-2f * Dt;
            }
        }
    }

    // 不修正 yaw 轴（四元数 q3 方向）
    kf.TempVectorData[3] = 0.0f;

    // 后验状态
    for (uint8_t i = 0; i < 6; i++)
    {
        kf.XhatData[i] = kf.XhatminusData[i] + kf.TempVectorData[i];
        if (!std::isfinite(kf.XhatData[i]))
        {
            return ARM_MATH_NANINF;
        }
    }
    return ARM_MATH_SUCCESS;
}

bool EKF::SetExternalRBoost(float scale)
{
    if (!std::isfinite(scale) || scale < 1.0f)
    {
        return false;
    }
    ExternalRBoost = scale;
    return true;
}

void EKF::SetFreezeUpdate(bool freeze)
{
    FreezeUpdate = freeze;
}

void EKF::GetQuaternion(float q[4]) const
{
    q[0] = Q[0];
    q[1] = Q[1];
    q[2] = Q[2];
    q[3] = Q[3];
}

void EKF::GetGyroBias(float bias[3]) const
{
    bias[0] = GyroBias[0];
    bias[1] = GyroBias[1];
    bias[2] = GyroBias[2];
}

void EKF::GetYawPitchRoll(float ypr[3]) const
{
    ypr[0] = Yaw;
    ypr[1] = Pitch;
    ypr[2] = Roll;
}

float EKF::GetYawTotalAngle() const
{
    return YawTotalAngle;
}

bool EKF::IsInitialized() const
{
    return Initialized;
}

bool EKF::IsConverged() const
{
    return ConvergeFlag;
}

bool EKF::IsStable() const
{
    return StableFlag;
}

arm_status EKF::GetLastStatus() const
{
    return LastStatus;
}

uint64_t EKF::GetUpdateCount() const
{
    return UpdateCount;
}

} // namespace alg_estimate
