#pragma once

#include "Defines.h"
#include "EigenCompat.h"
#include "SigmaPointSetSimplex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#ifndef MAZEMAP_UKF_USE_CMSIS_DSP
#if !defined(_MSC_VER) && (defined(CORE_TEENSY) || defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41))
#define MAZEMAP_UKF_USE_CMSIS_DSP 1
#else
#define MAZEMAP_UKF_USE_CMSIS_DSP 0
#endif
#endif

#if MAZEMAP_UKF_USE_CMSIS_DSP
#include "arm_math.h"
#endif

namespace MazeMap
{
    // Default no-op callback used when a UKF caller does not need per-sigma-point instrumentation.
    struct NoopUkfLoopHook
    {
        void operator()() const noexcept {}
    };

    namespace detail
    {
        template <int Rows, int Cols>
        struct UkfStorageOrder
        {
            static constexpr int value =
                (Rows == 1 && Cols != 1) ? Eigen::RowMajor : Eigen::ColMajor;
        };

        template <int Rows, int Cols>
        using UkfMatrix = Eigen::Matrix<float, Rows, Cols, UkfStorageOrder<Rows, Cols>::value>;

        struct UkfFloatOps
        {
            static void Fill(float* dst, int count, float value) noexcept
            {
#if MAZEMAP_UKF_USE_CMSIS_DSP
                arm_fill_f32(value, dst, static_cast<uint32_t>(count));
#else
                for (int index = 0; index < count; ++index)
                {
                    dst[index] = value;
                }
#endif
            }

            static void Copy(const float* src, float* dst, int count) noexcept
            {
#if MAZEMAP_UKF_USE_CMSIS_DSP
                arm_copy_f32(const_cast<float*>(src), dst, static_cast<uint32_t>(count));
#else
                for (int index = 0; index < count; ++index)
                {
                    dst[index] = src[index];
                }
#endif
            }

            static void Add(const float* lhs, const float* rhs, float* dst, int count) noexcept
            {
#if MAZEMAP_UKF_USE_CMSIS_DSP
                arm_add_f32(
                    const_cast<float*>(lhs),
                    const_cast<float*>(rhs),
                    dst,
                    static_cast<uint32_t>(count));
#else
                for (int index = 0; index < count; ++index)
                {
                    dst[index] = lhs[index] + rhs[index];
                }
#endif
            }

            static void Sub(const float* lhs, const float* rhs, float* dst, int count) noexcept
            {
#if MAZEMAP_UKF_USE_CMSIS_DSP
                arm_sub_f32(
                    const_cast<float*>(lhs),
                    const_cast<float*>(rhs),
                    dst,
                    static_cast<uint32_t>(count));
#else
                for (int index = 0; index < count; ++index)
                {
                    dst[index] = lhs[index] - rhs[index];
                }
#endif
            }

            static void Scale(const float* src, float scale, float* dst, int count) noexcept
            {
#if MAZEMAP_UKF_USE_CMSIS_DSP
                arm_scale_f32(
                    const_cast<float*>(src),
                    scale,
                    dst,
                    static_cast<uint32_t>(count));
#else
                for (int index = 0; index < count; ++index)
                {
                    dst[index] = src[index] * scale;
                }
#endif
            }

            static float Dot(const float* lhs, const float* rhs, int count) noexcept
            {
#if MAZEMAP_UKF_USE_CMSIS_DSP
                float result = 0.0f;
                arm_dot_prod_f32(
                    const_cast<float*>(lhs),
                    const_cast<float*>(rhs),
                    static_cast<uint32_t>(count),
                    &result);
                return result;
#else
                float result = 0.0f;
                for (int index = 0; index < count; ++index)
                {
                    result += lhs[index] * rhs[index];
                }
                return result;
#endif
            }

            static void Axpy(float alpha, const float* x, float* y, int count) noexcept
            {
                if (alpha == 0.0f)
                {
                    return;
                }

                for (int index = 0; index < count; ++index)
                {
                    y[index] += alpha * x[index];
                }
            }

            static void RankOneAccumulate(
                float* destinationColMajor,
                int rows,
                int cols,
                const float* lhs,
                const float* rhs,
                float alpha) noexcept
            {
                if (alpha == 0.0f)
                {
                    return;
                }

                for (int col = 0; col < cols; ++col)
                {
                    Axpy(alpha * rhs[col], lhs, destinationColMajor + (col * rows), rows);
                }
            }
        };
    }

    // Tunable unscented-transform weights for a square-root UKF with a fixed state dimension.
    template <int NState>
    struct SrUkfWeights
    {
        static constexpr int kSigmaCount = (2 * NState) + 1;

        float alpha = 0.3f;
        float beta = 2.0f;
        float kappa = 0.0f;

        float lambda() const noexcept
        {
            return (alpha * alpha * (static_cast<float>(NState) + kappa)) - static_cast<float>(NState);
        }

        float gamma() const noexcept
        {
            const float spread = static_cast<float>(NState) + lambda();
            return (spread > 0.0f) ? MazeMap::Math::Sqrtf(spread) : 0.0f;
        }

        float meanWeight(int index) const noexcept
        {
            if (index == 0)
            {
                return lambda() / (static_cast<float>(NState) + lambda());
            }

            return 0.5f / (static_cast<float>(NState) + lambda());
        }

        float covarianceWeight(int index) const noexcept
        {
            if (index == 0)
            {
                return meanWeight(0) + (1.0f - (alpha * alpha) + beta);
            }

            return meanWeight(index);
        }
    };

    // Square-root covariance helpers shared by the reusable UKF implementation and estimator-specific layers.
    template <int NState>
    struct SrUkfMath
    {
        using Vector = detail::UkfMatrix<NState, 1>;
        using Matrix = detail::UkfMatrix<NState, NState>;

        template <int Cols>
        using MatrixNxCols = detail::UkfMatrix<NState, Cols>;

        static Matrix Symmetrize(const Matrix& input) noexcept
        {
            return 0.5f * (input + input.transpose());
        }

        static Matrix RegularizeCovariance(const Matrix& covariance) noexcept
        {
            Matrix candidate = Symmetrize(covariance);
            if (!candidate.allFinite())
            {
                return Matrix::Identity() * 1.0e-3f;
            }

            const float scale = (std::max)(1.0f, candidate.diagonal().cwiseAbs().maxCoeff());
            const float minimumDiagonal = 1.0e-9f * scale;
            for (int row = 0; row < NState; ++row)
            {
                for (int col = 0; col < NState; ++col)
                {
                    if (!std::isfinite(candidate(row, col)))
                    {
                        candidate(row, col) = 0.0f;
                    }
                }

                if (!std::isfinite(candidate(row, row)) || (candidate(row, row) < minimumDiagonal))
                {
                    candidate(row, row) = minimumDiagonal;
                }
            }

            return Symmetrize(candidate);
        }

        static bool FactorCovariance(const Matrix& covariance, Matrix& sqrtCovariance) noexcept
        {
            Matrix candidate = RegularizeCovariance(covariance);
            Eigen::LLT<Matrix> llt;
            llt.compute(candidate);
            if (llt.info() == Eigen::Success)
            {
                sqrtCovariance = llt.matrixL();
                MakePositiveDiagonal(sqrtCovariance);
                return true;
            }

            float jitter = 1.0e-9f;
            const float scale = (std::max)(1.0f, candidate.diagonal().cwiseAbs().maxCoeff());
            for (int attempt = 0; attempt < 12; ++attempt)
            {
                Matrix regularized = candidate;
                regularized.diagonal().array() += jitter * scale;
                llt.compute(regularized);
                if (llt.info() == Eigen::Success)
                {
                    sqrtCovariance = llt.matrixL();
                    MakePositiveDiagonal(sqrtCovariance);
                    return true;
                }
                jitter *= 10.0f;
            }

            return false;
        }

        static void MakePositiveDiagonal(Matrix& lowerTriangular) noexcept
        {
            for (int col = 0; col < NState; ++col)
            {
                if (lowerTriangular(col, col) >= 0.0f)
                {
                    continue;
                }

                lowerTriangular.col(col) *= -1.0f;
            }
        }

        template <int Cols>
        static Matrix QrSquareRoot(const MatrixNxCols<Cols>& columns) noexcept
        {
            using TransposeMatrix = Eigen::Matrix<float, Cols, NState>;
            const TransposeMatrix transpose = columns.transpose();
            Eigen::HouseholderQR<TransposeMatrix> qr(transpose);
            TransposeMatrix qrPacked = qr.matrixQR();
            Eigen::Matrix<float, NState, NState> upper =
                qrPacked.template topLeftCorner<NState, NState>().template triangularView<Eigen::Upper>();

            for (int row = 0; row < NState; ++row)
            {
                if (upper(row, row) < 0.0f)
                {
                    upper.row(row) *= -1.0f;
                }
            }

            Matrix lower = upper.transpose();
            MakePositiveDiagonal(lower);
            return lower;
        }

        static bool CholUpdate(Matrix& lowerTriangular, Vector vector, float weight) noexcept
        {
            if (!(weight != 0.0f) || !std::isfinite(weight))
            {
                return true;
            }

            const float sign = (weight > 0.0f) ? 1.0f : -1.0f;
            vector *= MazeMap::Math::Sqrtf(std::fabs(weight));

            for (int k = 0; k < NState; ++k)
            {
                const float diagonal = lowerTriangular(k, k);
                if (!(diagonal > 0.0f) || !std::isfinite(diagonal))
                {
                    return false;
                }

                const float vk = vector(k);
                const float radialSquared = (diagonal * diagonal) + (sign * vk * vk);
                if (!(radialSquared > 0.0f) || !std::isfinite(radialSquared))
                {
                    return false;
                }

                const float radial = MazeMap::Math::Sqrtf(radialSquared);
                const float c = radial / diagonal;
                const float s = vk / diagonal;

                lowerTriangular(k, k) = radial;
                for (int row = k + 1; row < NState; ++row)
                {
                    const float previous = lowerTriangular(row, k);
                    if (sign > 0.0f)
                    {
                        lowerTriangular(row, k) = (previous + (s * vector(row))) / c;
                    }
                    else
                    {
                        lowerTriangular(row, k) = (previous - (s * vector(row))) / c;
                    }
                    vector(row) = (c * vector(row)) - (s * lowerTriangular(row, k));
                }
            }

            MakePositiveDiagonal(lowerTriangular);
            return true;
        }

        static float ComputeNis(
            const Eigen::Ref<const Vector>& innovation,
            const Matrix& innovationSqrt) noexcept
        {
            const Vector whitened =
                innovationSqrt.template triangularView<Eigen::Lower>().solve(innovation);
            return detail::UkfFloatOps::Dot(whitened.data(), whitened.data(), NState);
        }

        template <int M>
        static float ComputeNis(
            const Eigen::Matrix<float, M, 1>& innovation,
            const Eigen::Matrix<float, M, M>& innovationSqrt) noexcept
        {
            const Eigen::Matrix<float, M, 1> whitened =
                innovationSqrt.template triangularView<Eigen::Lower>().solve(innovation);
            return detail::UkfFloatOps::Dot(whitened.data(), whitened.data(), M);
        }
    };

    // Generic square-root unscented Kalman filter for a fixed state and control dimension.
    template <int NState, int NControl>
    class UKF
    {
    public:
        static constexpr int kSigmaCount = (2 * NState) + 1;

        enum class SigmaPointStrategy : uint8_t
        {
            Standard = 0U,
            Simplex = 1U
        };

        using StateVec = detail::UkfMatrix<NState, 1>;
        using StateMat = detail::UkfMatrix<NState, NState>;
        using ControlVec = detail::UkfMatrix<NControl, 1>;
        using SigmaMat = detail::UkfMatrix<NState, kSigmaCount>;
        using WeightVec = Eigen::Matrix<float, kSigmaCount, 1>;

        UKF() noexcept
            : _weights()
            , _state(StateVec::Zero())
            , _sqrtCovariance(StateMat::Identity() * 1.0e-3f)
            , _sqrtProcessNoise(StateMat::Identity() * 1.0e-3f)
            , _sigmaPoints(SigmaMat::Zero())
            , _sigmaMean(StateVec::Zero())
            , _predictionCacheValid(false)
            , _lastNis(0.0f)
            , _stateNormalizer(nullptr)
            , _sigmaPointStrategy(SigmaPointStrategy::Simplex)
            , _activeSigmaCount(kSigmaCount)
            , _meanWeights(WeightVec::Zero())
            , _covarianceWeights(WeightVec::Zero())
        {
            refreshSigmaPointWeights();
        }

        const StateVec& state() const noexcept { return _state; }
        const StateMat& sqrtCovariance() const noexcept { return _sqrtCovariance; }
        StateMat covariance() const noexcept
        {
            return (_sqrtCovariance * _sqrtCovariance.transpose());
        }
        float lastNis() const noexcept { return _lastNis; }
        int activeSigmaCount() const noexcept { return _activeSigmaCount; }
        SigmaPointStrategy sigmaPointStrategy() const noexcept { return _sigmaPointStrategy; }

        void setStateNormalizer(void (*normalizer)(StateVec&)) noexcept
        {
            _stateNormalizer = normalizer;
        }

        void setState(const StateVec& state, const StateMat& covariance) noexcept
        {
            _state = state;
            normalizeState(_state);
            StateMat sqrtCovariance = _sqrtCovariance;
            if (SrUkfMath<NState>::FactorCovariance(covariance, sqrtCovariance))
            {
                _sqrtCovariance = sqrtCovariance;
            }
            _predictionCacheValid = false;
        }

        void setStateSquareRootCovariance(const StateVec& state, const StateMat& sqrtCovariance) noexcept
        {
            _state = state;
            normalizeState(_state);
            _sqrtCovariance = sqrtCovariance;
            SrUkfMath<NState>::MakePositiveDiagonal(_sqrtCovariance);
            _predictionCacheValid = false;
        }

        void setProcessNoise(const StateMat& covariance) noexcept
        {
            StateMat sqrtNoise = _sqrtProcessNoise;
            if (SrUkfMath<NState>::FactorCovariance(covariance, sqrtNoise))
            {
                _sqrtProcessNoise = sqrtNoise;
            }
        }

        void setProcessNoiseSquareRoot(const StateMat& sqrtNoise) noexcept
        {
            _sqrtProcessNoise = sqrtNoise;
            SrUkfMath<NState>::MakePositiveDiagonal(_sqrtProcessNoise);
        }

        const SrUkfWeights<NState>& weights() const noexcept
        {
            return _weights;
        }

        void setWeights(const SrUkfWeights<NState>& weights) noexcept
        {
            _weights = weights;
            refreshSigmaPointWeights();
            _predictionCacheValid = false;
        }

        void setSigmaPointStrategy(SigmaPointStrategy strategy) noexcept
        {
            _sigmaPointStrategy = strategy;
            refreshSigmaPointWeights();
            _predictionCacheValid = false;
        }

        void setSimplexSigmaPointSetEnabled(bool enabled) noexcept
        {
            setSigmaPointStrategy(enabled ? SigmaPointStrategy::Simplex : SigmaPointStrategy::Standard);
        }

        template <typename ProcessFn>
        bool Predict(float dt, const ControlVec& control, ProcessFn&& processFn) noexcept
        {
            return Predict(dt, control, processFn, NoopUkfLoopHook{});
        }

        template <typename ProcessFn, typename LoopHook>
        bool Predict(float dt, const ControlVec& control, ProcessFn&& processFn, LoopHook&& loopHook) noexcept
        {
            const float gamma = (_sigmaPointStrategy == SigmaPointStrategy::Standard) ? _weights.gamma() : 0.0f;

            SigmaMat priorSigma;
            makeSigmaPoints(_state, _sqrtCovariance, gamma, priorSigma);

            SigmaMat predictedSigma;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                loopHook();
                predictedSigma.col(column) = processFn(priorSigma.col(column), control, dt);
                normalizeSigmaColumn(predictedSigma, column);
            }

            StateVec predictedMean = predictedSigma.col(0);
            StateVec meanDelta;
            detail::UkfFloatOps::Fill(meanDelta.data(), NState, 0.0f);
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                loopHook();
                const StateVec delta = subtractAndNormalizeState(predictedSigma.col(column), predictedSigma.col(0));
                detail::UkfFloatOps::Axpy(_meanWeights(column), delta.data(), meanDelta.data(), NState);
            }
            detail::UkfFloatOps::Axpy(1.0f, meanDelta.data(), predictedMean.data(), NState);
            normalizeState(predictedMean);

            detail::UkfMatrix<NState, (2 * NState) + NState> stacked;
            detail::UkfFloatOps::Fill(stacked.data(), stacked.rows() * stacked.cols(), 0.0f);
            int residualColumn = 0;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (_covarianceWeights(column) <= 0.0f)
                {
                    continue;
                }

                loopHook();
                const StateVec residual = subtractAndNormalizeState(predictedSigma.col(column), predictedMean);
                detail::UkfFloatOps::Scale(
                    residual.data(),
                    MazeMap::Math::Sqrtf(_covarianceWeights(column)),
                    stacked.col(residualColumn).data(),
                    NState);
                ++residualColumn;
            }
            stacked.template rightCols<NState>() = _sqrtProcessNoise;

            StateMat predictedSqrt = SrUkfMath<NState>::template QrSquareRoot<(2 * NState) + NState>(stacked);

            _state = predictedMean;
            _sqrtCovariance = predictedSqrt;
            _sigmaPoints = predictedSigma;
            _sigmaMean = predictedMean;
            _predictionCacheValid = true;
            return true;
        }

        template <int M, typename MeasurementFn>
        bool Update(
            const Eigen::Matrix<float, M, 1>& measurement,
            const Eigen::Matrix<float, M, M>& sqrtMeasurementNoise,
            float nisThreshold,
            MeasurementFn&& measurementFn) noexcept
        {
            return Update<M>(
                measurement,
                sqrtMeasurementNoise,
                nisThreshold,
                measurementFn,
                NoopUkfLoopHook{});
        }

        template <int M, typename MeasurementFn, typename LoopHook>
        bool Update(
            const Eigen::Matrix<float, M, 1>& measurement,
            const Eigen::Matrix<float, M, M>& sqrtMeasurementNoise,
            float nisThreshold,
            MeasurementFn&& measurementFn,
            LoopHook&& loopHook) noexcept
        {
            if (!_predictionCacheValid)
            {
                const float gamma = (_sigmaPointStrategy == SigmaPointStrategy::Standard) ? _weights.gamma() : 0.0f;
                makeSigmaPoints(_state, _sqrtCovariance, gamma, _sigmaPoints);
                _sigmaMean = _state;
            }

            detail::UkfMatrix<M, kSigmaCount> measurementSigma;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                loopHook();
                measurementSigma.col(column) = measurementFn(_sigmaPoints.col(column));
            }

            Eigen::Matrix<float, M, 1> predictedMeasurement = Eigen::Matrix<float, M, 1>::Zero();
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                loopHook();
                detail::UkfFloatOps::Axpy(
                    _meanWeights(column),
                    measurementSigma.col(column).data(),
                    predictedMeasurement.data(),
                    M);
            }

            detail::UkfMatrix<M, (2 * NState) + M> stacked;
            detail::UkfFloatOps::Fill(stacked.data(), stacked.rows() * stacked.cols(), 0.0f);
            int residualColumn = 0;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (_covarianceWeights(column) <= 0.0f)
                {
                    continue;
                }

                loopHook();
                const Eigen::Matrix<float, M, 1> residual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                detail::UkfFloatOps::Scale(
                    residual.data(),
                    MazeMap::Math::Sqrtf(_covarianceWeights(column)),
                    stacked.col(residualColumn).data(),
                    M);
                ++residualColumn;
            }
            stacked.template rightCols<M>() = sqrtMeasurementNoise;

            using MeasMat = detail::UkfMatrix<M, M>;
            MeasMat innovationSqrt =
                SrUkfMath<M>::template QrSquareRoot<(2 * NState) + M>(stacked);

            detail::UkfMatrix<NState, M> crossCovariance;
            detail::UkfFloatOps::Fill(crossCovariance.data(), NState * M, 0.0f);
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                loopHook();
                const StateVec stateResidual = subtractAndNormalizeState(_sigmaPoints.col(column), _sigmaMean);
                const Eigen::Matrix<float, M, 1> measurementResidual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                detail::UkfFloatOps::RankOneAccumulate(
                    crossCovariance.data(),
                    NState,
                    M,
                    stateResidual.data(),
                    measurementResidual.data(),
                    _covarianceWeights(column));
            }

            const Eigen::Matrix<float, M, NState> solvedLower =
                innovationSqrt.template triangularView<Eigen::Lower>().solve(crossCovariance.transpose());
            const Eigen::Matrix<float, M, NState> solvedUpper =
                innovationSqrt.transpose().template triangularView<Eigen::Upper>().solve(solvedLower);
            const detail::UkfMatrix<NState, M> kalmanGain = solvedUpper.transpose();

            const Eigen::Matrix<float, M, 1> innovation = subtractMeasurement<M>(measurement, predictedMeasurement);
            _lastNis = SrUkfMath<M>::ComputeNis(innovation, innovationSqrt);
            if (std::isfinite(nisThreshold) && (nisThreshold > 0.0f) && (_lastNis >= nisThreshold))
            {
                return false;
            }

            _state += kalmanGain * innovation;
            normalizeState(_state);

            StateMat updatedSqrt = _sqrtCovariance;
            const detail::UkfMatrix<NState, M> updateColumns =
                multiplyRightLowerTriangular<M>(kalmanGain, innovationSqrt);
            for (int column = 0; column < M; ++column)
            {
                loopHook();
                if (!SrUkfMath<NState>::CholUpdate(updatedSqrt, updateColumns.col(column), -1.0f))
                {
                    const StateMat posteriorCovariance = rebuildPosteriorCovariance(updateColumns);
                    if (!SrUkfMath<NState>::FactorCovariance(posteriorCovariance, updatedSqrt))
                    {
                        return false;
                    }
                    break;
                }
            }

            _sqrtCovariance = updatedSqrt;
            _predictionCacheValid = false;
            return true;
        }

    private:
        template <typename LeftDerived, typename RightDerived>
        StateVec subtractAndNormalizeState(
            const Eigen::MatrixBase<LeftDerived>& lhs,
            const Eigen::MatrixBase<RightDerived>& rhs) const noexcept
        {
            StateVec result;
            detail::UkfFloatOps::Sub(lhs.derived().data(), rhs.derived().data(), result.data(), NState);
            normalizeState(result);
            return result;
        }

        template <int M, typename LeftDerived, typename RightDerived>
        detail::UkfMatrix<M, 1> subtractMeasurement(
            const Eigen::MatrixBase<LeftDerived>& lhs,
            const Eigen::MatrixBase<RightDerived>& rhs) const noexcept
        {
            detail::UkfMatrix<M, 1> result;
            detail::UkfFloatOps::Sub(lhs.derived().data(), rhs.derived().data(), result.data(), M);
            return result;
        }

        template <int M>
        detail::UkfMatrix<NState, M> multiplyRightLowerTriangular(
            const detail::UkfMatrix<NState, M>& left,
            const detail::UkfMatrix<M, M>& lowerTriangular) const noexcept
        {
            detail::UkfMatrix<NState, M> result;
            detail::UkfFloatOps::Fill(result.data(), NState * M, 0.0f);
            for (int column = 0; column < M; ++column)
            {
                for (int sourceColumn = column; sourceColumn < M; ++sourceColumn)
                {
                    const float scale = lowerTriangular(sourceColumn, column);
                    if (scale != 0.0f)
                    {
                        detail::UkfFloatOps::Axpy(
                            scale,
                            left.col(sourceColumn).data(),
                            result.col(column).data(),
                            NState);
                    }
                }
            }
            return result;
        }

        void normalizeSigmaColumn(SigmaMat& sigmaPoints, int column) const noexcept
        {
            if (_stateNormalizer == nullptr)
            {
                return;
            }

            StateVec normalized = sigmaPoints.col(column);
            _stateNormalizer(normalized);
            detail::UkfFloatOps::Copy(normalized.data(), sigmaPoints.col(column).data(), NState);
        }

        StateMat processNoiseCovariance() const noexcept
        {
            return SrUkfMath<NState>::Symmetrize(_sqrtProcessNoise * _sqrtProcessNoise.transpose());
        }

        StateMat rebuildPredictedCovariance(const SigmaMat& predictedSigma, const StateVec& predictedMean) const noexcept
        {
            StateMat covariance = processNoiseCovariance();
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                const StateVec residual = subtractAndNormalizeState(predictedSigma.col(column), predictedMean);
                covariance += _covarianceWeights(column) * (residual * residual.transpose());
            }
            return SrUkfMath<NState>::Symmetrize(covariance);
        }

        template <int M>
        detail::UkfMatrix<M, M> rebuildInnovationCovariance(
            const detail::UkfMatrix<M, kSigmaCount>& measurementSigma,
            const Eigen::Matrix<float, M, 1>& predictedMeasurement,
            const Eigen::Matrix<float, M, M>& sqrtMeasurementNoise) const noexcept
        {
            using MeasMat = detail::UkfMatrix<M, M>;
            MeasMat covariance =
                SrUkfMath<M>::Symmetrize(sqrtMeasurementNoise * sqrtMeasurementNoise.transpose());
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                const Eigen::Matrix<float, M, 1> residual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                covariance += _covarianceWeights(column) * (residual * residual.transpose());
            }
            return SrUkfMath<M>::Symmetrize(covariance);
        }

        StateMat rebuildPosteriorCovariance(const Eigen::Ref<const Eigen::Matrix<float, NState, Eigen::Dynamic>>& updateColumns) const noexcept
        {
            StateMat covariance = this->covariance();
            covariance -= updateColumns * updateColumns.transpose();
            return (covariance);
        }

        void makeSigmaPoints(
            const StateVec& mean,
            const StateMat& sqrtCovariance,
            float gamma,
            SigmaMat& sigmaPoints) const noexcept
        {
            if (_sigmaPointStrategy == SigmaPointStrategy::Simplex)
            {
                SigmaPointSetSimplex::GenerateSigmaPoints<NState>(mean, sqrtCovariance, sigmaPoints);
                for (int column = 0; column < _activeSigmaCount; ++column)
                {
                    normalizeSigmaColumn(sigmaPoints, column);
                }
                return;
            }

            detail::UkfFloatOps::Copy(mean.data(), sigmaPoints.col(0).data(), NState);
            normalizeSigmaColumn(sigmaPoints, 0);

            StateVec columnDelta;
            for (int column = 0; column < NState; ++column)
            {
                detail::UkfFloatOps::Scale(
                    sqrtCovariance.col(column).data(),
                    gamma,
                    columnDelta.data(),
                    NState);
                detail::UkfFloatOps::Add(
                    mean.data(),
                    columnDelta.data(),
                    sigmaPoints.col(column + 1).data(),
                    NState);
                detail::UkfFloatOps::Sub(
                    mean.data(),
                    columnDelta.data(),
                    sigmaPoints.col(column + 1 + NState).data(),
                    NState);
                normalizeSigmaColumn(sigmaPoints, column + 1);
                normalizeSigmaColumn(sigmaPoints, column + 1 + NState);
            }
        }

        void refreshSigmaPointWeights() noexcept
        {
            _meanWeights.setZero();
            _covarianceWeights.setZero();

            if (_sigmaPointStrategy == SigmaPointStrategy::Simplex)
            {
                _activeSigmaCount = SigmaPointSetSimplex::ActiveSigmaCountForDimension(NState);
                SigmaPointSetSimplex::ComputeWeights<NState>(_meanWeights, _covarianceWeights);
                return;
            }

            _activeSigmaCount = kSigmaCount;
            for (int column = 0; column < kSigmaCount; ++column)
            {
                _meanWeights(column) = _weights.meanWeight(column);
                _covarianceWeights(column) = _weights.covarianceWeight(column);
            }
        }

        void normalizeState(StateVec& state) const noexcept
        {
            if (_stateNormalizer != nullptr)
            {
                _stateNormalizer(state);
            }
        }

        SrUkfWeights<NState> _weights;
        StateVec _state;
        StateMat _sqrtCovariance;
        StateMat _sqrtProcessNoise;
        SigmaMat _sigmaPoints;
        StateVec _sigmaMean;
        bool _predictionCacheValid;
        float _lastNis;
        void (*_stateNormalizer)(StateVec&);
        SigmaPointStrategy _sigmaPointStrategy;
        int _activeSigmaCount;
        WeightVec _meanWeights;
        WeightVec _covarianceWeights;
    };
}
