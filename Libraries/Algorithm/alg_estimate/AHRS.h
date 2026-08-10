#ifndef AHRS_H
#define AHRS_H

#include <cstdint>

namespace alg_estimate
{

/**
 * @brief Mahony AHRS 姿态解算（四阶 Runge-Kutta 积分）
 * @details 移植自 mc02 ins_AHRS.c 的 INS_AHRSupdate / INS_GetYawPitchRoll，
 *          算法行为与原实现保持一致：
 *          - PI 修正陀螺仪零漂 + RK4 四元数积分
 *          - 单位约定：陀螺仪 rad/s、加速度 g（与原实现一致）
 *          - 磁力计参数保留但当前恒传 0（尚未接入）
 *          - 采样周期 dt 由调用方显式传入（原实现内部依赖 DWT 计时）
 */
class AHRS
{
  public:
    struct Config
    {
        float Kp = 0.8f;  // PI 比例增益
        float Ki = 0.01f; // PI 积分增益
    };

    bool Init(const Config &config);

    /**
   * @brief 更新姿态
   * @param gx gy gz 陀螺仪角速度，rad/s
   * @param ax ay az 加速度，g
   * @param mx my mz 磁力计，uT（当前恒传 0）
   * @param dt 采样周期，s
   */
    bool Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

    void GetQuaternion(float q[4]) const;     // q = {q0, q1, q2, q3}
    void GetYawPitchRoll(float ypr[3]) const; // ypr = {Yaw, Pitch, Roll}，度
    float GetYawTotalAngle() const;           // 累计偏航角（跨 ±180 计数），度
    bool IsInitialized() const;

    /// 清零 PI 积分项（陀螺零偏重新校准时由调用方调用，对应原实现的 exInt/eyInt/ezInt 清零）
    void ResetIntegral();

    /// 完全复位（四元数、积分项、累计偏航角）
    void Reset();

  private:
    static void QuatDerivative(float q0,
                               float q1,
                               float q2,
                               float q3,
                               float gx,
                               float gy,
                               float gz,
                               float &dq0,
                               float &dq1,
                               float &dq2,
                               float &dq3);

    float Kp = 0.8f;
    float Ki = 0.01f;
    float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float Integral[3] = {0.0f, 0.0f, 0.0f}; // PI 积分项（exInt/eyInt/ezInt）
    float Yaw = 0.0f;
    float Pitch = 0.0f;
    float Roll = 0.0f;
    float YawTotalAngle = 0.0f;
    float YawAngleLast = 0.0f;
    int32_t YawRoundCount = 0;
    bool Initialized = false;
    bool YawFirstFlag = true;
};

} // namespace alg_estimate

#endif
