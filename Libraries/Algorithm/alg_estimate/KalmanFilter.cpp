#include "KalmanFilter.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <new>
#include <utility>

namespace
{

bool IsFinite(const alg_estimate::BufferView<float> &values)
{
    for (float value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

} // namespace

namespace alg_estimate
{

bool KalmanFilter::Init(uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    Initialized = false;
    MatStatus = ARM_MATH_ARGUMENT_ERROR;

    if (xhatSize == 0 || zSize == 0 || zSize > xhatSize)
    {
        return false;
    }

    const size_t xSize = xhatSize;
    const size_t zDimension = zSize;
    const size_t uDimension = uSize;
    const size_t xSquare = xSize * xSize;
    const size_t zSquare = zDimension * zDimension;
    const size_t xuSize = xSize * uDimension;
    const size_t xzSize = xSize * zDimension;
    const size_t floatCount =
        8U * xSize + 4U * zDimension + 2U * uDimension + 9U * xSquare + xuSize + 3U * xzSize + zSquare;
    const size_t byteCount = 3U * zDimension;

    std::unique_ptr<float[]> floatStorage(new (std::nothrow) float[floatCount]());
    std::unique_ptr<uint8_t[]> byteStorage(new (std::nothrow) uint8_t[byteCount]());
    if (floatStorage == nullptr || byteStorage == nullptr)
    {
        return false;
    }

    FloatStorage = std::move(floatStorage);
    ByteStorage = std::move(byteStorage);

    float *floatCursor = FloatStorage.get();
    FilteredValue.Reset(floatCursor, xSize);
    floatCursor += xSize;
    FilteredValueBackup.Reset(floatCursor, xSize);
    floatCursor += xSize;
    MeasuredVector.Reset(floatCursor, zDimension);
    floatCursor += zDimension;
    ControlVector.Reset(floatCursor, uDimension);
    floatCursor += uDimension;
    MeasurementDegree.Reset(floatCursor, zDimension);
    floatCursor += zDimension;
    MatRDiagonalElements.Reset(floatCursor, zDimension);
    floatCursor += zDimension;
    StateMinVariance.Reset(floatCursor, xSize);
    floatCursor += xSize;

    XhatData.Reset(floatCursor, xSize);
    floatCursor += xSize;
    XhatminusData.Reset(floatCursor, xSize);
    floatCursor += xSize;
    XhatBackupData.Reset(floatCursor, xSize);
    floatCursor += xSize;
    UData.Reset(floatCursor, uDimension);
    floatCursor += uDimension;
    ZData.Reset(floatCursor, zDimension);
    floatCursor += zDimension;

    PData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    PminusData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    PBackupData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    FData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    FtData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    BData.Reset(floatCursor, xuSize);
    floatCursor += xuSize;
    HData.Reset(floatCursor, xzSize);
    floatCursor += xzSize;
    HtData.Reset(floatCursor, xzSize);
    floatCursor += xzSize;
    QData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    RData.Reset(floatCursor, zSquare);
    floatCursor += zSquare;
    KData.Reset(floatCursor, xzSize);
    floatCursor += xzSize;
    SData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    TempMatrixData.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    TempMatrixData1.Reset(floatCursor, xSquare);
    floatCursor += xSquare;
    TempVectorData.Reset(floatCursor, xSize);
    floatCursor += xSize;
    TempVectorData1.Reset(floatCursor, xSize);

    uint8_t *byteCursor = ByteStorage.get();
    MeasurementMap.Reset(byteCursor, zDimension);
    byteCursor += zDimension;
    MeasurementValid.Reset(byteCursor, zDimension);
    byteCursor += zDimension;
    Temp.Reset(byteCursor, zDimension);

    XhatSize = xhatSize;
    USize = uSize;
    ZSize = zSize;
    MeasurementValidNum = 0;
    UseAutoAdjustment = false;
    SkipEq1 = false;
    SkipEq2 = false;
    SkipEq3 = false;
    SkipEq4 = false;
    SkipEq5 = false;
    UserFunc0 = nullptr;
    UserFunc1 = nullptr;
    UserFunc2 = nullptr;
    UserFunc3 = nullptr;
    UserFunc4 = nullptr;
    UserFunc5 = nullptr;
    UserFunc6 = nullptr;
    UserData = nullptr;
    arm_mat_init_f32(&Xhat, XhatSize, 1, XhatData.data());
    arm_mat_init_f32(&Xhatminus, XhatSize, 1, XhatminusData.data());

    U = {};
    B = {};
    if (USize != 0)
    {
        arm_mat_init_f32(&U, USize, 1, UData.data());
        arm_mat_init_f32(&B, XhatSize, USize, BData.data());
    }

    arm_mat_init_f32(&Z, ZSize, 1, ZData.data());

    arm_mat_init_f32(&P, XhatSize, XhatSize, PData.data());
    arm_mat_init_f32(&Pminus, XhatSize, XhatSize, PminusData.data());

    arm_mat_init_f32(&F, XhatSize, XhatSize, FData.data());
    arm_mat_init_f32(&Ft, XhatSize, XhatSize, FtData.data());

    arm_mat_init_f32(&H, ZSize, XhatSize, HData.data());
    arm_mat_init_f32(&Ht, XhatSize, ZSize, HtData.data());

    arm_mat_init_f32(&Q, XhatSize, XhatSize, QData.data());
    arm_mat_init_f32(&R, ZSize, ZSize, RData.data());
    arm_mat_init_f32(&K, XhatSize, ZSize, KData.data());

    arm_mat_init_f32(&S, XhatSize, XhatSize, SData.data());
    arm_mat_init_f32(&TempMatrix, XhatSize, XhatSize, TempMatrixData.data());
    arm_mat_init_f32(&TempMatrix1, XhatSize, XhatSize, TempMatrixData1.data());
    arm_mat_init_f32(&TempVector, XhatSize, 1, TempVectorData.data());
    arm_mat_init_f32(&TempVector1, XhatSize, 1, TempVectorData1.data());

    MatStatus = ARM_MATH_SUCCESS;
    Initialized = true;
    return true;
}

bool KalmanFilter::SetMeasurement(uint8_t index, float value, bool valid)
{
    if (!Initialized || index >= ZSize || (valid && !std::isfinite(value)))
    {
        return false;
    }

    MeasuredVector[index] = valid ? value : 0.0f;
    MeasurementValid[index] = valid ? 1U : 0U;
    return true;
}

arm_status KalmanFilter::Measure()
{
    if (!Initialized || MeasuredVector.size() != ZSize || ZData.size() != ZSize || ControlVector.size() != USize ||
        UData.size() != USize)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    if (UseAutoAdjustment)
    {
        const arm_status status = HKRAdjustment();
        if (status != ARM_MATH_SUCCESS)
        {
            return status;
        }
    }
    else
    {
        H.numRows = ZSize;
        H.numCols = XhatSize;
        Ht.numRows = XhatSize;
        Ht.numCols = ZSize;
        R.numRows = ZSize;
        R.numCols = ZSize;
        K.numRows = XhatSize;
        K.numCols = ZSize;
        Z.numRows = ZSize;
        Z.numCols = 1;
        std::copy(MeasuredVector.begin(), MeasuredVector.end(), ZData.begin());
        std::fill(MeasuredVector.begin(), MeasuredVector.end(), 0.0f);
        std::fill(MeasurementValid.begin(), MeasurementValid.end(), 0U);
    }

    if (USize != 0)
    {
        std::copy(ControlVector.begin(), ControlVector.end(), UData.begin());
    }
    return ARM_MATH_SUCCESS;
}

arm_status KalmanFilter::XhatMinusUpdate()
{
    if (SkipEq1)
    {
        return ARM_MATH_SUCCESS;
    }

    if (USize == 0)
    {
        return arm_mat_mult_f32(&F, &Xhat, &Xhatminus);
    }

    TempVector.numRows = XhatSize;
    TempVector.numCols = 1;
    arm_status status = arm_mat_mult_f32(&F, &Xhat, &TempVector);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempVector1.numRows = XhatSize;
    TempVector1.numCols = 1;
    status = arm_mat_mult_f32(&B, &U, &TempVector1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    return arm_mat_add_f32(&TempVector, &TempVector1, &Xhatminus);
}

arm_status KalmanFilter::PminusUpdate()
{
    if (SkipEq2)
    {
        return ARM_MATH_SUCCESS;
    }

    arm_status status = arm_mat_trans_f32(&F, &Ft);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    status = arm_mat_mult_f32(&F, &P, &Pminus);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempMatrix.numRows = Pminus.numRows;
    TempMatrix.numCols = Ft.numCols;
    status = arm_mat_mult_f32(&Pminus, &Ft, &TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    return arm_mat_add_f32(&TempMatrix, &Q, &Pminus);
}

arm_status KalmanFilter::SetK()
{
    if (SkipEq3)
    {
        return ARM_MATH_SUCCESS;
    }

    arm_status status = arm_mat_trans_f32(&H, &Ht);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempMatrix.numRows = H.numRows;
    TempMatrix.numCols = Pminus.numCols;
    status = arm_mat_mult_f32(&H, &Pminus, &TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempMatrix1.numRows = TempMatrix.numRows;
    TempMatrix1.numCols = Ht.numCols;
    status = arm_mat_mult_f32(&TempMatrix, &Ht, &TempMatrix1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    S.numRows = R.numRows;
    S.numCols = R.numCols;
    status = arm_mat_add_f32(&TempMatrix1, &R, &S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    status = arm_mat_inverse_f32(&S, &TempMatrix1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempMatrix.numRows = Pminus.numRows;
    TempMatrix.numCols = Ht.numCols;
    status = arm_mat_mult_f32(&Pminus, &Ht, &TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    return arm_mat_mult_f32(&TempMatrix, &TempMatrix1, &K);
}

arm_status KalmanFilter::XhatUpdate()
{
    if (SkipEq4)
    {
        return ARM_MATH_SUCCESS;
    }

    TempVector.numRows = H.numRows;
    TempVector.numCols = 1;
    arm_status status = arm_mat_mult_f32(&H, &Xhatminus, &TempVector);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempVector1.numRows = Z.numRows;
    TempVector1.numCols = 1;
    status = arm_mat_sub_f32(&Z, &TempVector, &TempVector1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempVector.numRows = K.numRows;
    TempVector.numCols = 1;
    status = arm_mat_mult_f32(&K, &TempVector1, &TempVector);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    return arm_mat_add_f32(&Xhatminus, &TempVector, &Xhat);
}

arm_status KalmanFilter::PUpdate()
{
    if (SkipEq5)
    {
        return ARM_MATH_SUCCESS;
    }

    const uint16_t stateSize = XhatSize;
    const uint16_t measurementSize = H.numRows;

    // P = (I - KH)P-(I - KH)^T + KRK^T.
    TempMatrix.numRows = stateSize;
    TempMatrix.numCols = stateSize;
    arm_status status = arm_mat_mult_f32(&K, &H, &TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    for (uint16_t row = 0; row < stateSize; row++)
    {
        for (uint16_t column = 0; column < stateSize; column++)
        {
            const uint16_t index = row * stateSize + column;
            const float identity = row == column ? 1.0f : 0.0f;
            TempMatrixData[index] = identity - TempMatrixData[index];
        }
    }

    TempMatrix1.numRows = stateSize;
    TempMatrix1.numCols = stateSize;
    status = arm_mat_mult_f32(&TempMatrix, &Pminus, &TempMatrix1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    S.numRows = stateSize;
    S.numCols = stateSize;
    status = arm_mat_trans_f32(&TempMatrix, &S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    status = arm_mat_mult_f32(&TempMatrix1, &S, &P);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempMatrix.numRows = stateSize;
    TempMatrix.numCols = measurementSize;
    status = arm_mat_mult_f32(&K, &R, &TempMatrix);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    S.numRows = measurementSize;
    S.numCols = stateSize;
    status = arm_mat_trans_f32(&K, &S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    TempMatrix1.numRows = stateSize;
    TempMatrix1.numCols = stateSize;
    status = arm_mat_mult_f32(&TempMatrix, &S, &TempMatrix1);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    const uint16_t covarianceSize = stateSize * stateSize;
    for (uint16_t i = 0; i < covarianceSize; i++)
    {
        PData[i] += TempMatrixData1[i];
    }

    for (uint16_t row = 0; row < stateSize; row++)
    {
        for (uint16_t column = row + 1U; column < stateSize; column++)
        {
            const uint16_t upperIndex = row * stateSize + column;
            const uint16_t lowerIndex = column * stateSize + row;
            const float symmetricValue = 0.5f * (PData[upperIndex] + PData[lowerIndex]);
            PData[upperIndex] = symmetricValue;
            PData[lowerIndex] = symmetricValue;
        }
    }

    return ARM_MATH_SUCCESS;
}

bool KalmanFilter::Update()
{
    if (!Initialized)
    {
        MatStatus = ARM_MATH_ARGUMENT_ERROR;
        return false;
    }

    std::copy(XhatData.begin(), XhatData.end(), XhatBackupData.begin());
    std::copy(PData.begin(), PData.end(), PBackupData.begin());
    std::copy(FilteredValue.begin(), FilteredValue.end(), FilteredValueBackup.begin());
    const bool skipEq5Backup = SkipEq5;

    const auto fail = [this, skipEq5Backup](arm_status status)
    {
        std::copy(XhatBackupData.begin(), XhatBackupData.end(), XhatData.begin());
        std::copy(PBackupData.begin(), PBackupData.end(), PData.begin());
        std::copy(FilteredValueBackup.begin(), FilteredValueBackup.end(), FilteredValue.begin());
        SkipEq5 = skipEq5Backup;
        MatStatus = status;
        return false;
    };

    arm_status status = Measure();
    if (status != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }
    if (UserFunc0 != nullptr && (status = UserFunc0(*this)) != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }

    status = XhatMinusUpdate();
    if (status != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }
    if (UserFunc1 != nullptr && (status = UserFunc1(*this)) != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }

    status = PminusUpdate();
    if (status != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }
    if (UserFunc2 != nullptr && (status = UserFunc2(*this)) != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }

    if (MeasurementValidNum != 0 || !UseAutoAdjustment)
    {
        status = SetK();
        if (status != ARM_MATH_SUCCESS)
        {
            return fail(status);
        }
        if (UserFunc3 != nullptr && (status = UserFunc3(*this)) != ARM_MATH_SUCCESS)
        {
            return fail(status);
        }

        status = XhatUpdate();
        if (status != ARM_MATH_SUCCESS)
        {
            return fail(status);
        }
        if (UserFunc4 != nullptr && (status = UserFunc4(*this)) != ARM_MATH_SUCCESS)
        {
            return fail(status);
        }

        status = PUpdate();
        if (status != ARM_MATH_SUCCESS)
        {
            return fail(status);
        }
    }
    else
    {
        std::copy(XhatminusData.begin(), XhatminusData.end(), XhatData.begin());
        std::copy(PminusData.begin(), PminusData.end(), PData.begin());
    }

    if (UserFunc5 != nullptr && (status = UserFunc5(*this)) != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }

    for (uint8_t i = 0; i < XhatSize; i++)
    {
        if (!std::isfinite(StateMinVariance[i]) || StateMinVariance[i] < 0.0f)
        {
            return fail(ARM_MATH_ARGUMENT_ERROR);
        }
        if (PData[i * XhatSize + i] < StateMinVariance[i])
        {
            PData[i * XhatSize + i] = StateMinVariance[i];
        }
    }

    if (!IsFinite(XhatData) || !IsFinite(PData))
    {
        return fail(ARM_MATH_NANINF);
    }

    std::copy(XhatData.begin(), XhatData.end(), FilteredValue.begin());
    if (UserFunc6 != nullptr && (status = UserFunc6(*this)) != ARM_MATH_SUCCESS)
    {
        return fail(status);
    }

    MatStatus = ARM_MATH_SUCCESS;
    return true;
}

const float *KalmanFilter::GetFilteredValue() const
{
    return Initialized ? FilteredValue.data() : nullptr;
}

arm_status KalmanFilter::GetLastStatus() const
{
    return MatStatus;
}

bool KalmanFilter::IsInitialized() const
{
    return Initialized;
}

arm_status KalmanFilter::HKRAdjustment()
{
    if (MeasurementMap.size() != ZSize || MeasurementValid.size() != ZSize || MeasurementDegree.size() != ZSize ||
        MatRDiagonalElements.size() != ZSize || Temp.size() != ZSize)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    MeasurementValidNum = 0;
    std::copy(MeasuredVector.begin(), MeasuredVector.end(), ZData.begin());
    std::fill(RData.begin(), RData.end(), 0.0f);
    std::fill(HData.begin(), HData.end(), 0.0f);

    const auto clearMeasurement = [this]()
    {
        std::fill(MeasuredVector.begin(), MeasuredVector.end(), 0.0f);
        std::fill(MeasurementValid.begin(), MeasurementValid.end(), 0U);
    };

    for (uint8_t i = 0; i < ZSize; i++)
    {
        if (MeasurementValid[i] == 0U)
        {
            continue;
        }
        if (!std::isfinite(ZData[i]) || !std::isfinite(MeasurementDegree[i]) ||
            !std::isfinite(MatRDiagonalElements[i]) || MatRDiagonalElements[i] < 0.0f)
        {
            clearMeasurement();
            return ARM_MATH_ARGUMENT_ERROR;
        }
        if (MeasurementMap[i] == 0 || MeasurementMap[i] > XhatSize)
        {
            clearMeasurement();
            return ARM_MATH_ARGUMENT_ERROR;
        }

        ZData[MeasurementValidNum] = ZData[i];
        Temp[MeasurementValidNum] = i;
        HData[XhatSize * MeasurementValidNum + MeasurementMap[i] - 1U] = MeasurementDegree[i];
        MeasurementValidNum++;
    }

    clearMeasurement();

    for (uint8_t i = 0; i < MeasurementValidNum; i++)
    {
        RData[i * MeasurementValidNum + i] = MatRDiagonalElements[Temp[i]];
    }

    H.numRows = MeasurementValidNum;
    H.numCols = XhatSize;
    Ht.numRows = XhatSize;
    Ht.numCols = MeasurementValidNum;
    R.numRows = MeasurementValidNum;
    R.numCols = MeasurementValidNum;
    K.numRows = XhatSize;
    K.numCols = MeasurementValidNum;
    Z.numRows = MeasurementValidNum;
    Z.numCols = 1;
    return ARM_MATH_SUCCESS;
}

} // namespace alg_estimate
