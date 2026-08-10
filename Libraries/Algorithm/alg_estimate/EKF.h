#ifndef EKF_H
#define EKF_H

#include "KalmanFilter.h"
#include <cstdint>

namespace alg_estimate
{

class EKF
{
  public:
    struct Config
    {
        float ProcessNoise1 = 1.0e-3f; // 四元数过程噪声 Q1
        float ProcessNoise2 = 1.0e-4f; // 陀螺零偏过程噪声 Q2
        float MeasureNoise = 1.0f;     // 加速度计量测噪声 R
        float Lambda = 1.0f;           // 渐消因子（0~1，1 表示无渐消）
        float AccLpfCoef = 0.0f;       // 内部加速度低通系数（>0 启用）
    };

    bool Init(const float initQuaternion[4], const Config &config);

    bool Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    bool SetExternalRBoost(float scale);

    void SetFreezeUpdate(bool freeze);

    void GetQuaternion(float q[4]) const;
    void GetGyroBias(float bias[3]) const;
    void GetYawPitchRoll(float ypr[3]) const; // ypr = {Yaw, Pitch, Roll}（度）
    float GetYawTotalAngle() const;
    bool IsInitialized() const;
    bool IsConverged() const;
    bool IsStable() const;
    arm_status GetLastStatus() const;
    uint64_t GetUpdateCount() const;

  private:
    static arm_status UserFunc0(KalmanFilter &kf); // Observe：矩阵快照
    static arm_status UserFunc1(KalmanFilter &kf); // 四元数精确预测 + F 线性化 + P 渐消
    static arm_status UserFunc2(KalmanFilter &kf); // 观测雅可比
    static arm_status UserFunc3(KalmanFilter &kf); // 卡方检验 + K 计算 + 后验更新

    arm_status Observe();
    arm_status PredictAndLinearize();
    arm_status PredictQuaternionExact();
    arm_status SetH();
    arm_status XhatUpdate();

    KalmanFilter Kf;
    arm_matrix_instance_f32 ChiSquare;
    float ChiSquareData[1] = {0.0f};

    float PSnap[36] = {};
    float KSnap[18] = {};
    float HSnap[18] = {};

    float Q[4] = {};
    float GyroBias[3] = {};
    float Gyro[3] = {};  // 去零偏后的角速度
    float Accel[3] = {}; // 低通后的加速度
    float OrientationCosine[3] = {};
    float GyroNorm = 0.0f;
    float AccelNorm = 0.0f;
    float AdaptiveGainScale = 1.0f;
    float Roll = 0.0f;
    float Pitch = 0.0f;
    float Yaw = 0.0f;
    float YawTotalAngle = 0.0f;
    float YawAngleLast = 0.0f;
    int16_t YawRoundCount = 0;

    float Q1 = 0.0f;
    float Q2 = 0.0f;
    float R = 0.0f;
    float Lambda = 1.0f;
    float ChiSquareThreshold = 1.0e-3f;
    float AccLpfCoef = 0.0f;
    float ExternalRBoost = 1.0f;
    float Dt = 0.0f;
    bool Initialized = false;
    bool ConvergeFlag = false;
    bool StableFlag = false;
    bool FreezeUpdate = false;
    arm_status LastStatus = ARM_MATH_SUCCESS;
    uint64_t ErrorCount = 0;
    uint64_t UpdateCount = 0;
};

} // namespace alg_estimate

#endif
