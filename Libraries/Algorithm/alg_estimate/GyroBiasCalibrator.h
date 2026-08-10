#ifndef GYRO_BIAS_CALIBRATOR_H
#define GYRO_BIAS_CALIBRATOR_H

#include <cstdint>

namespace alg_estimate
{

enum class GyroBiasUpdateResult : uint8_t
{
    InvalidInput = 0,
    Collecting,
    JustCalibrated,
};

class GyroBiasCalibrator
{
  public:
    static constexpr uint16_t maxWindow = 300;

    struct Config
    {
        uint16_t WindowLength = 100;
        float VarianceThreshold = 0.02f;
        uint16_t MinStableCount = 100;
    };

    bool Init(const Config &config);

    GyroBiasUpdateResult Update(const float gyroDps[3]);

    bool IsInitialized() const;

    bool IsCalibrated() const;

    void GetBias(float bias[3]) const;

    void GetVariance(float variance[3]) const;

    void GetAverage(float average[3]) const;

    void Reset();

  private:
    double Fill[3][maxWindow] = {};
    double Total[3] = {};
    double SqrTotal[3] = {};
    float Bias[3] = {0.0f, 0.0f, 0.0f};
    float Variance[3] = {0.0f, 0.0f, 0.0f};
    float Average[3] = {0.0f, 0.0f, 0.0f};
    uint16_t Window = 100;
    uint16_t Count = 0;
    uint16_t StableCount = 0;
    float VarianceThreshold = 0.02f;
    uint16_t MinStableCount = 100;
    bool WindowFilled = false;
    bool Calibrated = false;
    bool Initialized = false;
};

} // namespace alg_estimate

#endif
