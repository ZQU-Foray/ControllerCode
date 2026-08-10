#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "arm_math.h"
#include <cstddef>
#include <cstdint>
#include <memory>

namespace alg_estimate
{

class KalmanFilter;

template <typename T> class BufferView
{
  public:
    BufferView() = default;
    BufferView(const BufferView &) = delete;
    BufferView &operator=(const BufferView &) = delete;
    BufferView(BufferView &&) = delete;
    BufferView &operator=(BufferView &&) = delete;

    T *data()
    {
        return Data;
    }

    const T *data() const
    {
        return Data;
    }

    T *begin()
    {
        return Data;
    }

    const T *begin() const
    {
        return Data;
    }

    T *end()
    {
        return Size == 0U ? Data : Data + Size;
    }

    const T *end() const
    {
        return Size == 0U ? Data : Data + Size;
    }

    std::size_t size() const
    {
        return Size;
    }

    T &operator[](std::size_t index)
    {
        return Data[index];
    }

    const T &operator[](std::size_t index) const
    {
        return Data[index];
    }

  private:
    friend class KalmanFilter;

    void Reset(T *data, std::size_t size)
    {
        Data = data;
        Size = size;
    }

    T *Data = nullptr;
    std::size_t Size = 0U;
};

class KalmanFilter
{
  public:
    typedef arm_status (*UserFunc)(KalmanFilter &kf);

    KalmanFilter() = default;
    KalmanFilter(const KalmanFilter &) = delete;
    KalmanFilter &operator=(const KalmanFilter &) = delete;
    KalmanFilter(KalmanFilter &&) = delete;
    KalmanFilter &operator=(KalmanFilter &&) = delete;

    bool Init(uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

    // MeasurementValid is consumed by the next Update when auto adjustment is enabled.
    bool SetMeasurement(uint8_t index, float value, bool valid = true);

    arm_status Measure();

    arm_status XhatMinusUpdate();

    arm_status PminusUpdate();

    arm_status SetK();

    arm_status XhatUpdate();

    arm_status PUpdate();

    bool Update();

    const float *GetFilteredValue() const;

    arm_status GetLastStatus() const;

    bool IsInitialized() const;

    uint8_t XhatSize = 0;
    uint8_t USize = 0;
    uint8_t ZSize = 0;

    bool UseAutoAdjustment = false;
    uint8_t MeasurementValidNum = 0;

    bool SkipEq1 = false;
    bool SkipEq2 = false;
    bool SkipEq3 = false;
    bool SkipEq4 = false;
    bool SkipEq5 = false;

    arm_status MatStatus = ARM_MATH_SUCCESS;

    UserFunc UserFunc0 = nullptr;
    UserFunc UserFunc1 = nullptr;
    UserFunc UserFunc2 = nullptr;
    UserFunc UserFunc3 = nullptr;
    UserFunc UserFunc4 = nullptr;
    UserFunc UserFunc5 = nullptr;
    UserFunc UserFunc6 = nullptr;

    void *UserData = nullptr;

    BufferView<float> FilteredValue;
    BufferView<float> MeasuredVector;
    BufferView<float> ControlVector;
    BufferView<uint8_t> MeasurementMap;
    BufferView<uint8_t> MeasurementValid;
    BufferView<float> MeasurementDegree;
    BufferView<float> MatRDiagonalElements;
    BufferView<float> StateMinVariance;
    BufferView<uint8_t> Temp;

    arm_matrix_instance_f32 Xhat{};
    arm_matrix_instance_f32 Xhatminus{};
    arm_matrix_instance_f32 U{};
    arm_matrix_instance_f32 Z{};
    arm_matrix_instance_f32 P{};
    arm_matrix_instance_f32 Pminus{};
    arm_matrix_instance_f32 F{}, Ft{};
    arm_matrix_instance_f32 B{};
    arm_matrix_instance_f32 H{}, Ht{};
    arm_matrix_instance_f32 Q{};
    arm_matrix_instance_f32 R{};
    arm_matrix_instance_f32 K{};
    arm_matrix_instance_f32 S{};
    arm_matrix_instance_f32 TempMatrix{};
    arm_matrix_instance_f32 TempMatrix1{};
    arm_matrix_instance_f32 TempVector{};
    arm_matrix_instance_f32 TempVector1{};

    BufferView<float> XhatData;
    BufferView<float> XhatminusData;
    BufferView<float> UData;
    BufferView<float> ZData;
    BufferView<float> PData;
    BufferView<float> PminusData;
    BufferView<float> FData;
    BufferView<float> FtData;
    BufferView<float> BData;
    BufferView<float> HData;
    BufferView<float> HtData;
    BufferView<float> QData;
    BufferView<float> RData;
    BufferView<float> KData;
    BufferView<float> SData;
    BufferView<float> TempMatrixData;
    BufferView<float> TempMatrixData1;
    BufferView<float> TempVectorData;
    BufferView<float> TempVectorData1;

  private:
    arm_status HKRAdjustment();

    bool Initialized = false;
    std::unique_ptr<float[]> FloatStorage;
    std::unique_ptr<uint8_t[]> ByteStorage;
    BufferView<float> XhatBackupData;
    BufferView<float> PBackupData;
    BufferView<float> FilteredValueBackup;
};

} // namespace alg_estimate

#endif
