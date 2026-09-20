#ifndef LIBRARIES_ALGORITHM_ALG_ESTIMATE_IMU_BODY_MAPPING_HPP
#define LIBRARIES_ALGORITHM_ALG_ESTIMATE_IMU_BODY_MAPPING_HPP

#include "Libraries/Device/bmi088/Bmi088Accel.hpp"
#include "Libraries/Device/bmi088/Bmi088Gyro.hpp"
#include "Libraries/Device/bmi088/Bmi088SampleQueue.hpp"
#include <array>
#include <cstdint>

namespace alg_estimate
{

/**
 * @brief BMI088 原始样本到机体坐标系 SI 向量的确定性映射层。
 *
 * 本层只做两件被允许的确定性操作（见 EKF 重建计划 §1.2.3）：
 *   1. 原始整数到 SI 单位的固定比例换算；
 *   2. 固定坐标轴置换与符号映射。
 *
 * 本层没有任何状态、历史、均值、限幅或平滑：同一输入永远得到逐位相同的输出，
 * 输入数组与样本记录都不会被修改。
 *
 * 单位契约（计划 D11）：本层输出统一 SI —— 角速度 rad/s、加速度 m/s^2。
 * 设备层只提供 °/s（Bmi088Gyro::ToDps）与 m/s^2（Bmi088Accel::ToMps2），
 * 传感器原始计数是 ±32768 满量程的 int16。
 */

/**
 * @brief 传感器坐标系到机体坐标系的固定有符号轴置换。
 * @note 语义为 `body[i] = sign[i] * sensor[source[i]]`：source 是 {0,1,2} 的
 *       置换，sign 取 +1 或 -1。这里描述的是“机体轴 i 由传感器哪根轴、以什么
 *       符号提供”，与旋转矩阵的行写法等价。
 */
struct AxisMapping
{
    std::array<std::uint8_t, 3> source{};
    std::array<std::int8_t, 3> sign{};
};

/**
 * @brief 编译期校验轴映射是否合法。
 * @param mapping 待校验映射。
 * @return source 为 {0,1,2} 的置换、每个符号为 ±1、且行列式为 +1 时返回 true。
 * @note 行列式为 +1 排除了镜像映射（例如把 Z 轴取反而 X/Y 不变）：这种映射无法
 *       用右手系刚体旋转表示，会把陀螺与加速度计一起变成左手系，是欧拉角符号
 *       错误的常见来源，因此在编译期直接拒绝，避免用输出端翻符号来打补丁。
 */
[[nodiscard]] constexpr bool IsValidAxisMapping(const AxisMapping &mapping) noexcept
{
    std::int32_t inversions = 0;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        if (mapping.source[axis] > 2U)
        {
            return false;
        }
        if (mapping.sign[axis] != 1 && mapping.sign[axis] != -1)
        {
            return false;
        }
        for (std::size_t other = axis + 1U; other < 3U; ++other)
        {
            if (mapping.source[axis] == mapping.source[other])
            {
                return false;
            }
            if (mapping.source[axis] > mapping.source[other])
            {
                ++inversions;
            }
        }
    }

    std::int32_t determinant = static_cast<std::int32_t>(mapping.sign[0]) * static_cast<std::int32_t>(mapping.sign[1]) *
                               static_cast<std::int32_t>(mapping.sign[2]);
    if ((inversions % 2) != 0)
    {
        determinant = -determinant;
    }
    return determinant == 1;
}

/**
 * @brief BMI088 在 DM_MC02 上的实际安装映射。
 * @note 当前取值沿用上一轮板上验证的安装结论：传感器 Z 轴朝上，传感器轴与机体轴
 *       恒等映射。该结论**尚未**按 EKF 重建计划阶段 3 的要求用六面静止数据重新
 *       冻结，因此这里只是唯一可改点，不是最终定稿值。
 * @note 修改时只改这一处：非法置换或镜像映射会在编译期被 static_assert 拒绝，
 *       而 Tests/ImuMapping 的契约断言会失败，要求同步复核测试。
 */
inline constexpr AxisMapping Bmi088SensorToBody{{0U, 1U, 2U}, {1, 1, 1}};
static_assert(IsValidAxisMapping(Bmi088SensorToBody), "BMI088 安装映射必须是有符号轴置换且行列式为 +1");

/**
 * @brief 量程契约：换算常量只能与锁定的传感器配置同时成立。
 * @note 陀螺锁定 ±2000 °/s，加速度计锁定 ±3 g。一旦有人改动量程寄存器常量而忘记
 *       同步换算，这里会立即编译失败，而不是让姿态解算拿到错误标度（对照官方示例
 *       把 ±6 g 灵敏度常量当默认值的隐患）。
 */
static_assert(device::Bmi088Gyro::RangeRegisterValue == device::Bmi088Gyro::GyroRange2000Dps,
              "BMI088 陀螺量程不再是 ±2000 °/s，陀螺换算与 EKF 标度假设需要同步复核");
static_assert(device::Bmi088Accel::RangeRegisterValue == device::Bmi088Accel::AccelRange3g,
              "BMI088 加速度计量程不再是 ±3 g，加速度换算与 EKF 标度假设需要同步复核");

/**
 * @brief 陀螺原始计数换算为 SI 角速度。
 * @param raw 陀螺仪原始计数值。
 * @return 角速度，单位 rad/s。
 * @note 实现为“设备层 °/s 换算后再乘 degToRad”，保证与本工程既有的 ToDps 结果
 *       逐位一致，不引入第二套标度常量。
 */
[[nodiscard]] float GyroRawToRadiansPerSecond(std::int16_t raw) noexcept;

/**
 * @brief 按给定映射把传感器坐标系三轴向量搬到机体坐标系。
 * @param mapping 固定轴映射。
 * @param sensor 传感器坐标系输入，顺序为 x、y、z。
 * @param body 机体坐标系输出，顺序为 x、y、z。
 * @note 允许 body 与 sensor 为同一数组，内部先复制输入再写出结果。
 */
void MapSensorVectorToBody(const AxisMapping &mapping, const float (&sensor)[3], float (&body)[3]) noexcept;

/**
 * @brief 陀螺样本换算并映射为机体系角速度。
 * @param record 驱动收割到的原始样本记录，只读。
 * @param body 机体坐标系输出，单位 rad/s，顺序为 x、y、z。
 * @param mapping 固定轴映射。
 */
void GyroRecordToBody(const device::Bmi088SampleRecord &record,
                      float (&body)[3],
                      const AxisMapping &mapping = Bmi088SensorToBody) noexcept;

/**
 * @brief 加速度计样本换算并映射为机体系加速度。
 * @param record 驱动收割到的原始样本记录，只读。
 * @param body 机体坐标系输出，单位 m/s^2，顺序为 x、y、z。
 * @param mapping 固定轴映射。
 */
void AccelRecordToBody(const device::Bmi088SampleRecord &record,
                       float (&body)[3],
                       const AxisMapping &mapping = Bmi088SensorToBody) noexcept;

} // namespace alg_estimate

#endif
