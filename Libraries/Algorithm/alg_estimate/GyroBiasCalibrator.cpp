#include "GyroBiasCalibrator.h"
#include <cmath>

namespace alg_estimate
{

bool GyroBiasCalibrator::Init(const Config &config)
{
    Window = 100;
    VarianceThreshold = 0.02f;
    MinStableCount = 100;
    Initialized = false;
    Reset();

    if (config.WindowLength == 0 || config.WindowLength > maxWindow || !std::isfinite(config.VarianceThreshold) ||
        config.VarianceThreshold <= 0.0f || config.MinStableCount == 0)
    {
        return false;
    }

    Window = config.WindowLength;
    VarianceThreshold = config.VarianceThreshold;
    MinStableCount = config.MinStableCount;
    Initialized = true;
    return true;
}

void GyroBiasCalibrator::Reset()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint16_t j = 0; j < maxWindow; j++)
        {
            Fill[i][j] = 0.0;
        }
        Total[i] = 0.0;
        SqrTotal[i] = 0.0;
        Bias[i] = 0.0f;
        Variance[i] = 0.0f;
        Average[i] = 0.0f;
    }
    Count = 0;
    StableCount = 0;
    WindowFilled = false;
    Calibrated = false;
}

GyroBiasUpdateResult GyroBiasCalibrator::Update(const float gyroDps[3])
{
    if (!Initialized || gyroDps == nullptr || !std::isfinite(gyroDps[0]) || !std::isfinite(gyroDps[1]) ||
        !std::isfinite(gyroDps[2]))
    {
        return GyroBiasUpdateResult::InvalidInput;
    }

    if (!WindowFilled)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            Fill[i][Count] = gyroDps[i];
            Total[i] += gyroDps[i];
            SqrTotal[i] += gyroDps[i] * gyroDps[i];
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            Total[i] -= Fill[i][Count];
            SqrTotal[i] -= Fill[i][Count] * Fill[i][Count];
            Fill[i][Count] = gyroDps[i];
            Total[i] += Fill[i][Count];
            SqrTotal[i] += Fill[i][Count] * Fill[i][Count];
        }
    }

    Count++;
    if (Count >= Window)
    {
        Count = 0;
        WindowFilled = true;
    }

    if (!WindowFilled)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            Variance[i] = 0.0f;
            Average[i] = 0.0f;
        }
        StableCount = 0;
        return GyroBiasUpdateResult::Collecting;
    }

    const double sampleCount = Window;
    for (uint8_t i = 0; i < 3; i++)
    {
        Average[i] = static_cast<float>(Total[i] / sampleCount);
        double variance = (SqrTotal[i] - Total[i] * Total[i] / sampleCount) / sampleCount;
        if (variance < 0.0)
        {
            variance = 0.0;
        }
        Variance[i] = static_cast<float>(variance);
    }

    const bool stable =
        Variance[0] < VarianceThreshold && Variance[1] < VarianceThreshold && Variance[2] < VarianceThreshold;
    if (!stable)
    {
        StableCount = 0;
        return GyroBiasUpdateResult::Collecting;
    }

    if (StableCount < MinStableCount)
    {
        StableCount++;
    }
    if (StableCount < MinStableCount)
    {
        return GyroBiasUpdateResult::Collecting;
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        Bias[i] = Average[i];
    }
    StableCount = 0;
    Calibrated = true;
    return GyroBiasUpdateResult::JustCalibrated;
}

bool GyroBiasCalibrator::IsInitialized() const
{
    return Initialized;
}

bool GyroBiasCalibrator::IsCalibrated() const
{
    return Calibrated;
}

void GyroBiasCalibrator::GetBias(float bias[3]) const
{
    bias[0] = Bias[0];
    bias[1] = Bias[1];
    bias[2] = Bias[2];
}

void GyroBiasCalibrator::GetVariance(float variance[3]) const
{
    variance[0] = Variance[0];
    variance[1] = Variance[1];
    variance[2] = Variance[2];
}

void GyroBiasCalibrator::GetAverage(float average[3]) const
{
    average[0] = Average[0];
    average[1] = Average[1];
    average[2] = Average[2];
}

} // namespace alg_estimate
