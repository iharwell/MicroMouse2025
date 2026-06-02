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

        UKF() noexcept
            : _ownedState(Eigen::Matrix<float, NState, 1>::Zero())
            , _ownedSqrtCovariance(Eigen::Matrix<float, NState, NState>::Identity() * 1.0e-3f)
            , _weightAlpha(0.01f)
            , _weightBeta(2.0f)
            , _weightKappa(0.0f)
            , _state(_ownedState)
            , _sqrtCovariance(_ownedSqrtCovariance)
            , _sqrtProcessNoise(Eigen::Matrix<float, NState, NState>::Identity() * 1.0e-3f)
            , _sigmaPoints(Eigen::Matrix<float, NState, kSigmaCount>::Zero())
            , _sigmaMean(Eigen::Matrix<float, NState, 1>::Zero())
            , _predictionCacheValid(false)
            , _lastNis(0.0f)
            , _stateNormalizer(nullptr)
            , _sigmaPointStrategy(SigmaPointStrategy::Simplex)
            , _activeSigmaCount(kSigmaCount)
            , _meanWeights(Eigen::Matrix<float, kSigmaCount, 1>::Zero())
            , _covarianceWeights(Eigen::Matrix<float, kSigmaCount, 1>::Zero())
        {
            refreshSigmaPointWeights();
        }

        UKF(Eigen::Matrix<float, NState, 1>& externalState, Eigen::Matrix<float, NState, NState>& externalSqrtCovariance) noexcept
            : _ownedState(Eigen::Matrix<float, NState, 1>::Zero())
            , _ownedSqrtCovariance(Eigen::Matrix<float, NState, NState>::Identity() * 1.0e-3f)
            , _weightAlpha(0.3f)
            , _weightBeta(2.0f)
            , _weightKappa(0.0f)
            , _state(externalState)
            , _sqrtCovariance(externalSqrtCovariance)
            , _sqrtProcessNoise(Eigen::Matrix<float, NState, NState>::Identity() * 1.0e-3f)
            , _sigmaPoints(Eigen::Matrix<float, NState, kSigmaCount>::Zero())
            , _sigmaMean(Eigen::Matrix<float, NState, 1>::Zero())
            , _predictionCacheValid(false)
            , _lastNis(0.0f)
            , _stateNormalizer(nullptr)
            , _sigmaPointStrategy(SigmaPointStrategy::Simplex)
            , _activeSigmaCount(kSigmaCount)
            , _meanWeights(Eigen::Matrix<float, kSigmaCount, 1>::Zero())
            , _covarianceWeights(Eigen::Matrix<float, kSigmaCount, 1>::Zero())
        {
            refreshSigmaPointWeights();
        }

        const Eigen::Matrix<float, NState, 1>& state() const noexcept { return _state; }
        const Eigen::Matrix<float, NState, NState>& sqrtCovariance() const noexcept { return _sqrtCovariance; }
        Eigen::Matrix<float, NState, NState> covariance() const noexcept
        {
            return (_sqrtCovariance * _sqrtCovariance.transpose());
        }
        static bool FactorCovariance(
            const Eigen::Matrix<float, NState, NState>& covariance,
            Eigen::Matrix<float, NState, NState>& sqrtCovariance) noexcept
        {
            return FactorCovarianceForDimension<NState>(covariance, sqrtCovariance);
        }
        float variance(int stateIndex) const noexcept
        {
            if ((stateIndex < 0) || (stateIndex >= NState))
            {
                return 0.0f;
            }

            float value = 0.0f;
            for (int column = 0; column <= stateIndex; ++column)
            {
                const float term = _sqrtCovariance(stateIndex, column);
                value += term * term;
            }
            return value;
        }
        bool floorVariance(int stateIndex, float minimumVariance) noexcept
        {
            if ((stateIndex < 0) ||
                (stateIndex >= NState) ||
                !std::isfinite(minimumVariance) ||
                !(minimumVariance > 0.0f))
            {
                return false;
            }

            const float currentVariance = variance(stateIndex);
            if (!std::isfinite(currentVariance))
            {
                return false;
            }

            if (currentVariance >= minimumVariance)
            {
                return false;
            }

            const float deltaVariance = minimumVariance - currentVariance;
            if (!std::isfinite(deltaVariance) || !(deltaVariance > 0.0f))
            {
                return false;
            }

            Eigen::Matrix<float, NState, 1> updateVector = Eigen::Matrix<float, NState, 1>::Zero();
            updateVector(stateIndex) = MazeMap::Math::Sqrtf(deltaVariance);
            if (!CholUpdate<NState>(_sqrtCovariance, updateVector, 1.0f))
            {
                Eigen::Matrix<float, NState, NState> repairedCovariance = covariance();
                repairedCovariance(stateIndex, stateIndex) =
                    (std::max)(repairedCovariance(stateIndex, stateIndex), minimumVariance);
                Eigen::Matrix<float, NState, NState> repairedSqrtCovariance = _sqrtCovariance;
                if (!FactorCovariance(repairedCovariance, repairedSqrtCovariance))
                {
                    return false;
                }
                _sqrtCovariance = repairedSqrtCovariance;
            }

            _predictionCacheValid = false;
            return true;
        }
        float lastNis() const noexcept { return _lastNis; }
        int activeSigmaCount() const noexcept { return _activeSigmaCount; }
        SigmaPointStrategy sigmaPointStrategy() const noexcept { return _sigmaPointStrategy; }

        void setStateNormalizer(void (*normalizer)(Eigen::Matrix<float, NState, 1>&)) noexcept
        {
            _stateNormalizer = normalizer;
        }

        void setState(const Eigen::Matrix<float, NState, 1>& state, const Eigen::Matrix<float, NState, NState>& covariance) noexcept
        {
            _state = state;
            normalizeState(_state);
            Eigen::Matrix<float, NState, NState> sqrtCovariance = _sqrtCovariance;
            if (FactorCovariance(covariance, sqrtCovariance))
            {
                _sqrtCovariance = sqrtCovariance;
            }
            _predictionCacheValid = false;
        }

        void setStateSquareRootCovariance(const Eigen::Matrix<float, NState, 1>& state, const Eigen::Matrix<float, NState, NState>& sqrtCovariance) noexcept
        {
            _state = state;
            normalizeState(_state);
            _sqrtCovariance = sqrtCovariance;
            MakePositiveDiagonal<NState>(_sqrtCovariance);
            _predictionCacheValid = false;
        }

        void setProcessNoise(const Eigen::Matrix<float, NState, NState>& covariance) noexcept
        {
            Eigen::Matrix<float, NState, NState> sqrtNoise = _sqrtProcessNoise;
            if (FactorCovariance(covariance, sqrtNoise))
            {
                _sqrtProcessNoise = sqrtNoise;
            }
        }

        void setProcessNoiseSquareRoot(const Eigen::Matrix<float, NState, NState>& sqrtNoise) noexcept
        {
            _sqrtProcessNoise = sqrtNoise;
            MakePositiveDiagonal<NState>(_sqrtProcessNoise);
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
        bool Predict(float dt, const Eigen::Matrix<float, NControl, 1>& control, ProcessFn&& processFn) noexcept
        {
            const float gamma = (_sigmaPointStrategy == SigmaPointStrategy::Standard) ? WeightGamma() : 0.0f;

            Eigen::Matrix<float, NState, kSigmaCount> priorSigma;
            makeSigmaPoints(_state, _sqrtCovariance, gamma, priorSigma);

            Eigen::Matrix<float, NState, kSigmaCount> predictedSigma;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                predictedSigma.col(column) = processFn(priorSigma.col(column), control, dt);
                normalizeSigmaColumn(predictedSigma, column);
            }

            Eigen::Matrix<float, NState, 1> predictedMean = predictedSigma.col(0);
            Eigen::Matrix<float, NState, 1> meanDelta;
            Fill(meanDelta.data(), NState, 0.0f);
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                const Eigen::Matrix<float, NState, 1> delta = subtractAndNormalizeState(predictedSigma.col(column), predictedSigma.col(0));
                Axpy(_meanWeights(column), delta.data(), meanDelta.data(), NState);
            }
            Axpy(1.0f, meanDelta.data(), predictedMean.data(), NState);
            normalizeState(predictedMean);

            Eigen::Matrix<float, NState, (2 * NState) + NState> stacked;
            Fill(stacked.data(), static_cast<int>(stacked.rows() * stacked.cols()), 0.0f);
            int residualColumn = 0;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (_covarianceWeights(column) <= 0.0f)
                {
                    continue;
                }

                const Eigen::Matrix<float, NState, 1> residual = subtractAndNormalizeState(predictedSigma.col(column), predictedMean);
                Scale(
                    residual.data(),
                    MazeMap::Math::Sqrtf(_covarianceWeights(column)),
                    stacked.col(residualColumn).data(),
                    NState);
                ++residualColumn;
            }
            stacked.template rightCols<NState>() = _sqrtProcessNoise;

            Eigen::Matrix<float, NState, NState> predictedSqrt =
                QrSquareRoot<NState, (2 * NState) + NState>(stacked);

            _state = predictedMean;
            _sqrtCovariance = predictedSqrt;
            _sigmaPoints = predictedSigma;
            _sigmaMean = predictedMean;
            _predictionCacheValid = true;
            return true;
        }

        bool Predict(
            float dt,
            const Eigen::Matrix<float, NControl, 1>& control,
            void* processData,
            Eigen::Matrix<float, NState, 1> (*processFn)(void*, const Eigen::Matrix<float, NState, 1>&, const Eigen::Matrix<float, NControl, 1>&, float) noexcept,
            void* loopHookData = nullptr,
            void (*loopHook)(void*) noexcept = nullptr) noexcept
        {
            if (processFn == nullptr)
            {
                return false;
            }

            return predictImpl(dt, control, processData, processFn, loopHookData, loopHook);
        }

        template <int M, typename MeasurementFn>
        bool Update(
            const Eigen::Matrix<float, M, 1>& measurement,
            const Eigen::Matrix<float, M, M>& sqrtMeasurementNoise,
            float nisThreshold,
            MeasurementFn&& measurementFn) noexcept
        {
            if (!_predictionCacheValid)
            {
                const float gamma = (_sigmaPointStrategy == SigmaPointStrategy::Standard) ? WeightGamma() : 0.0f;
                makeSigmaPoints(_state, _sqrtCovariance, gamma, _sigmaPoints);
                _sigmaMean = _state;
            }

            Eigen::Matrix<float, M, kSigmaCount> measurementSigma;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                measurementSigma.col(column) = measurementFn(_sigmaPoints.col(column));
            }

            Eigen::Matrix<float, M, 1> predictedMeasurement = Eigen::Matrix<float, M, 1>::Zero();
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                Axpy(
                    _meanWeights(column),
                    measurementSigma.col(column).data(),
                    predictedMeasurement.data(),
                    M);
            }

            Eigen::Matrix<float, M, (2 * NState) + M> stacked;
            Fill(stacked.data(), static_cast<int>(stacked.rows() * stacked.cols()), 0.0f);
            int residualColumn = 0;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (_covarianceWeights(column) <= 0.0f)
                {
                    continue;
                }

                const Eigen::Matrix<float, M, 1> residual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                Scale(
                    residual.data(),
                    MazeMap::Math::Sqrtf(_covarianceWeights(column)),
                    stacked.col(residualColumn).data(),
                    M);
                ++residualColumn;
            }
            stacked.template rightCols<M>() = sqrtMeasurementNoise;

            Eigen::Matrix<float, M, M> innovationSqrt =
                QrSquareRoot<M, (2 * NState) + M>(stacked);

            Eigen::Matrix<float, NState, M> crossCovariance;
            Fill(crossCovariance.data(), NState * M, 0.0f);
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                const Eigen::Matrix<float, NState, 1> stateResidual = subtractAndNormalizeState(_sigmaPoints.col(column), _sigmaMean);
                const Eigen::Matrix<float, M, 1> measurementResidual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                RankOneAccumulate(
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
            const Eigen::Matrix<float, NState, M> kalmanGain = solvedUpper.transpose();

            const Eigen::Matrix<float, M, 1> innovation = subtractMeasurement<M>(measurement, predictedMeasurement);
            _lastNis = ComputeNis<M>(innovation, innovationSqrt);
            if (std::isfinite(nisThreshold) && (nisThreshold > 0.0f) && (_lastNis >= nisThreshold))
            {
                return false;
            }

            Eigen::Matrix<float, NState, 1> updatedState = _state + (kalmanGain * innovation);
            normalizeState(updatedState);

            Eigen::Matrix<float, NState, NState> updatedSqrt = _sqrtCovariance;
            const Eigen::Matrix<float, NState, M> updateColumns =
                multiplyRightLowerTriangular<M>(kalmanGain, innovationSqrt);
            for (int column = 0; column < M; ++column)
            {
                if (!CholUpdate<NState>(updatedSqrt, updateColumns.col(column), -1.0f))
                {
                    const Eigen::Matrix<float, NState, NState> posteriorCovariance = rebuildPosteriorCovariance(updateColumns);
                    if (!FactorCovariance(posteriorCovariance, updatedSqrt))
                    {
                        return false;
                    }
                    break;
                }
            }

            _state = updatedState;
            _sqrtCovariance = updatedSqrt;
            _predictionCacheValid = false;
            return true;
        }

        template <int M>
        bool Update(
            const Eigen::Matrix<float, M, 1>& measurement,
            const Eigen::Matrix<float, M, M>& sqrtMeasurementNoise,
            float nisThreshold,
            void* measurementData,
            Eigen::Matrix<float, M, 1> (*measurementFn)(void*, const Eigen::Matrix<float, NState, 1>&) noexcept) noexcept
        {
            if (measurementFn == nullptr)
            {
                return false;
            }

            return updateImpl<M>(
                measurement,
                sqrtMeasurementNoise,
                nisThreshold,
                measurementData,
                measurementFn);
        }

    private:
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

        float WeightLambda() const noexcept
        {
            return (_weightAlpha * _weightAlpha * (static_cast<float>(NState) + _weightKappa)) - static_cast<float>(NState);
        }

        float WeightGamma() const noexcept
        {
            const float spread = static_cast<float>(NState) + WeightLambda();
            return (spread > 0.0f) ? MazeMap::Math::Sqrtf(spread) : 0.0f;
        }

        float MeanWeight(const int index) const noexcept
        {
            if (index == 0)
            {
                return WeightLambda() / (static_cast<float>(NState) + WeightLambda());
            }

            return 0.5f / (static_cast<float>(NState) + WeightLambda());
        }

        float CovarianceWeight(const int index) const noexcept
        {
            if (index == 0)
            {
                return MeanWeight(0) + (1.0f - (_weightAlpha * _weightAlpha) + _weightBeta);
            }

            return MeanWeight(index);
        }

        template <int Dimension>
        static Eigen::Matrix<float, Dimension, Dimension> Symmetrize(
            const Eigen::Matrix<float, Dimension, Dimension>& input) noexcept
        {
            return 0.5f * (input + input.transpose());
        }

        template <int Dimension>
        static Eigen::Matrix<float, Dimension, Dimension> RegularizeCovariance(
            const Eigen::Matrix<float, Dimension, Dimension>& covariance) noexcept
        {
            Eigen::Matrix<float, Dimension, Dimension> candidate = Symmetrize<Dimension>(covariance);
            if (!candidate.allFinite())
            {
                return Eigen::Matrix<float, Dimension, Dimension>::Identity() * 1.0e-3f;
            }

            const float scale = (std::max)(1.0f, candidate.diagonal().cwiseAbs().maxCoeff());
            const float minimumDiagonal = 1.0e-9f * scale;
            for (int row = 0; row < Dimension; ++row)
            {
                for (int col = 0; col < Dimension; ++col)
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

            return Symmetrize<Dimension>(candidate);
        }

        template <int Dimension>
        static bool FactorCovarianceForDimension(
            const Eigen::Matrix<float, Dimension, Dimension>& covariance,
            Eigen::Matrix<float, Dimension, Dimension>& sqrtCovariance) noexcept
        {
            Eigen::Matrix<float, Dimension, Dimension> candidate = RegularizeCovariance<Dimension>(covariance);
            Eigen::LLT<Eigen::Matrix<float, Dimension, Dimension>> llt;
            llt.compute(candidate);
            if (llt.info() == Eigen::Success)
            {
                sqrtCovariance = llt.matrixL();
                MakePositiveDiagonal<Dimension>(sqrtCovariance);
                return true;
            }

            float jitter = 1.0e-9f;
            const float scale = (std::max)(1.0f, candidate.diagonal().cwiseAbs().maxCoeff());
            for (int attempt = 0; attempt < 12; ++attempt)
            {
                Eigen::Matrix<float, Dimension, Dimension> regularized = candidate;
                regularized.diagonal().array() += jitter * scale;
                llt.compute(regularized);
                if (llt.info() == Eigen::Success)
                {
                    sqrtCovariance = llt.matrixL();
                    MakePositiveDiagonal<Dimension>(sqrtCovariance);
                    return true;
                }
                jitter *= 10.0f;
            }

            return false;
        }

        template <int Dimension>
        static void MakePositiveDiagonal(Eigen::Matrix<float, Dimension, Dimension>& lowerTriangular) noexcept
        {
            for (int col = 0; col < Dimension; ++col)
            {
                if (lowerTriangular(col, col) >= 0.0f)
                {
                    continue;
                }

                lowerTriangular.col(col) *= -1.0f;
            }
        }

        template <int Dimension, int Cols>
        static Eigen::Matrix<float, Dimension, Dimension> QrSquareRoot(
            const Eigen::Matrix<float, Dimension, Cols>& columns) noexcept
        {
            const Eigen::Matrix<float, Cols, Dimension> transpose = columns.transpose();
            Eigen::HouseholderQR<Eigen::Matrix<float, Cols, Dimension>> qr(transpose);
            Eigen::Matrix<float, Cols, Dimension> qrPacked = qr.matrixQR();
            Eigen::Matrix<float, Dimension, Dimension> upper =
                qrPacked.template topLeftCorner<Dimension, Dimension>().template triangularView<Eigen::Upper>();

            for (int row = 0; row < Dimension; ++row)
            {
                if (upper(row, row) < 0.0f)
                {
                    upper.row(row) *= -1.0f;
                }
            }

            Eigen::Matrix<float, Dimension, Dimension> lower = upper.transpose();
            MakePositiveDiagonal<Dimension>(lower);
            return lower;
        }

        template <int Dimension>
        static bool CholUpdate(
            Eigen::Matrix<float, Dimension, Dimension>& lowerTriangular,
            Eigen::Matrix<float, Dimension, 1> vector,
            float weight) noexcept
        {
            if (!(weight != 0.0f) || !std::isfinite(weight))
            {
                return true;
            }

            const float sign = (weight > 0.0f) ? 1.0f : -1.0f;
            vector *= MazeMap::Math::Sqrtf(std::fabs(weight));

            for (int k = 0; k < Dimension; ++k)
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
                for (int row = k + 1; row < Dimension; ++row)
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

            MakePositiveDiagonal<Dimension>(lowerTriangular);
            return true;
        }

        template <int Dimension>
        static float ComputeNis(
            const Eigen::Matrix<float, Dimension, 1>& innovation,
            const Eigen::Matrix<float, Dimension, Dimension>& innovationSqrt) noexcept
        {
            const Eigen::Matrix<float, Dimension, 1> whitened =
                innovationSqrt.template triangularView<Eigen::Lower>().solve(innovation);
            return Dot(whitened.data(), whitened.data(), Dimension);
        }

        bool predictImpl(
            float dt,
            const Eigen::Matrix<float, NControl, 1>& control,
            void* processData,
            Eigen::Matrix<float, NState, 1> (*processFn)(
                void*,
                const Eigen::Matrix<float, NState, 1>&,
                const Eigen::Matrix<float, NControl, 1>&,
                float) noexcept,
            void* loopHookData,
            void (*loopHook)(void*) noexcept) noexcept
        {
            const float gamma = (_sigmaPointStrategy == SigmaPointStrategy::Standard) ? WeightGamma() : 0.0f;

            Eigen::Matrix<float, NState, kSigmaCount> priorSigma;
            makeSigmaPoints(_state, _sqrtCovariance, gamma, priorSigma);

            Eigen::Matrix<float, NState, kSigmaCount> predictedSigma;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (loopHook != nullptr)
                {
                    loopHook(loopHookData);
                }
                predictedSigma.col(column) =
                    processFn(processData, priorSigma.col(column), control, dt);
                normalizeSigmaColumn(predictedSigma, column);
            }

            Eigen::Matrix<float, NState, 1> predictedMean = predictedSigma.col(0);
            Eigen::Matrix<float, NState, 1> meanDelta;
            Fill(meanDelta.data(), NState, 0.0f);
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (loopHook != nullptr)
                {
                    loopHook(loopHookData);
                }
                const Eigen::Matrix<float, NState, 1> delta = subtractAndNormalizeState(predictedSigma.col(column), predictedSigma.col(0));
                Axpy(_meanWeights(column), delta.data(), meanDelta.data(), NState);
            }
            Axpy(1.0f, meanDelta.data(), predictedMean.data(), NState);
            normalizeState(predictedMean);

            Eigen::Matrix<float, NState, (2 * NState) + NState> stacked;
            Fill(stacked.data(), static_cast<int>(stacked.rows() * stacked.cols()), 0.0f);
            int residualColumn = 0;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (_covarianceWeights(column) <= 0.0f)
                {
                    continue;
                }

                if (loopHook != nullptr)
                {
                    loopHook(loopHookData);
                }
                const Eigen::Matrix<float, NState, 1> residual = subtractAndNormalizeState(predictedSigma.col(column), predictedMean);
                Scale(
                    residual.data(),
                    MazeMap::Math::Sqrtf(_covarianceWeights(column)),
                    stacked.col(residualColumn).data(),
                    NState);
                ++residualColumn;
            }
            stacked.template rightCols<NState>() = _sqrtProcessNoise;

            Eigen::Matrix<float, NState, NState> predictedSqrt =
                QrSquareRoot<NState, (2 * NState) + NState>(stacked);

            _state = predictedMean;
            _sqrtCovariance = predictedSqrt;
            _sigmaPoints = predictedSigma;
            _sigmaMean = predictedMean;
            _predictionCacheValid = true;
            return true;
        }

        template <int M>
        bool updateImpl(
            const Eigen::Matrix<float, M, 1>& measurement,
            const Eigen::Matrix<float, M, M>& sqrtMeasurementNoise,
            float nisThreshold,
            void* measurementData,
            Eigen::Matrix<float, M, 1> (*measurementFn)(
                void*,
                const Eigen::Matrix<float, NState, 1>&) noexcept) noexcept
        {
            if (!_predictionCacheValid)
            {
                const float gamma = (_sigmaPointStrategy == SigmaPointStrategy::Standard) ? WeightGamma() : 0.0f;
                makeSigmaPoints(_state, _sqrtCovariance, gamma, _sigmaPoints);
                _sigmaMean = _state;
            }

            Eigen::Matrix<float, M, kSigmaCount> measurementSigma;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                measurementSigma.col(column) =
                    measurementFn(measurementData, _sigmaPoints.col(column));
            }

            Eigen::Matrix<float, M, 1> predictedMeasurement = Eigen::Matrix<float, M, 1>::Zero();
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                Axpy(
                    _meanWeights(column),
                    measurementSigma.col(column).data(),
                    predictedMeasurement.data(),
                    M);
            }

            Eigen::Matrix<float, M, (2 * NState) + M> stacked;
            Fill(stacked.data(), static_cast<int>(stacked.rows() * stacked.cols()), 0.0f);
            int residualColumn = 0;
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                if (_covarianceWeights(column) <= 0.0f)
                {
                    continue;
                }

                const Eigen::Matrix<float, M, 1> residual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                Scale(
                    residual.data(),
                    MazeMap::Math::Sqrtf(_covarianceWeights(column)),
                    stacked.col(residualColumn).data(),
                    M);
                ++residualColumn;
            }
            stacked.template rightCols<M>() = sqrtMeasurementNoise;

            Eigen::Matrix<float, M, M> innovationSqrt =
                QrSquareRoot<M, (2 * NState) + M>(stacked);

            Eigen::Matrix<float, NState, M> crossCovariance;
            Fill(crossCovariance.data(), NState * M, 0.0f);
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                const Eigen::Matrix<float, NState, 1> stateResidual = subtractAndNormalizeState(_sigmaPoints.col(column), _sigmaMean);
                const Eigen::Matrix<float, M, 1> measurementResidual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                RankOneAccumulate(
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
            const Eigen::Matrix<float, NState, M> kalmanGain = solvedUpper.transpose();

            const Eigen::Matrix<float, M, 1> innovation = subtractMeasurement<M>(measurement, predictedMeasurement);
            _lastNis = ComputeNis<M>(innovation, innovationSqrt);
            if (std::isfinite(nisThreshold) && (nisThreshold > 0.0f) && (_lastNis >= nisThreshold))
            {
                return false;
            }

            Eigen::Matrix<float, NState, 1> updatedState = _state + (kalmanGain * innovation);
            normalizeState(updatedState);

            Eigen::Matrix<float, NState, NState> updatedSqrt = _sqrtCovariance;
            const Eigen::Matrix<float, NState, M> updateColumns =
                multiplyRightLowerTriangular<M>(kalmanGain, innovationSqrt);
            for (int column = 0; column < M; ++column)
            {
                if (!CholUpdate<NState>(updatedSqrt, updateColumns.col(column), -1.0f))
                {
                    const Eigen::Matrix<float, NState, NState> posteriorCovariance = rebuildPosteriorCovariance(updateColumns);
                    if (!FactorCovariance(posteriorCovariance, updatedSqrt))
                    {
                        return false;
                    }
                    break;
                }
            }

            _state = updatedState;
            _sqrtCovariance = updatedSqrt;
            _predictionCacheValid = false;
            return true;
        }

        template <typename LeftDerived, typename RightDerived>
        Eigen::Matrix<float, NState, 1> subtractAndNormalizeState(
            const Eigen::MatrixBase<LeftDerived>& lhs,
            const Eigen::MatrixBase<RightDerived>& rhs) const noexcept
        {
            Eigen::Matrix<float, NState, 1> result;
            Sub(lhs.derived().data(), rhs.derived().data(), result.data(), NState);
            normalizeState(result);
            return result;
        }

        template <int M, typename LeftDerived, typename RightDerived>
        Eigen::Matrix<float, M, 1> subtractMeasurement(
            const Eigen::MatrixBase<LeftDerived>& lhs,
            const Eigen::MatrixBase<RightDerived>& rhs) const noexcept
        {
            Eigen::Matrix<float, M, 1> result;
            Sub(lhs.derived().data(), rhs.derived().data(), result.data(), M);
            return result;
        }

        template <int M>
        Eigen::Matrix<float, NState, M> multiplyRightLowerTriangular(
            const Eigen::Matrix<float, NState, M>& left,
            const Eigen::Matrix<float, M, M>& lowerTriangular) const noexcept
        {
            Eigen::Matrix<float, NState, M> result;
            Fill(result.data(), NState * M, 0.0f);
            for (int column = 0; column < M; ++column)
            {
                for (int sourceColumn = column; sourceColumn < M; ++sourceColumn)
                {
                    const float scale = lowerTriangular(sourceColumn, column);
                    if (scale != 0.0f)
                    {
                        Axpy(
                            scale,
                            left.col(sourceColumn).data(),
                            result.col(column).data(),
                            NState);
                    }
                }
            }
            return result;
        }

        void normalizeSigmaColumn(Eigen::Matrix<float, NState, kSigmaCount>& sigmaPoints, int column) const noexcept
        {
            if (_stateNormalizer == nullptr)
            {
                return;
            }

            Eigen::Matrix<float, NState, 1> normalized = sigmaPoints.col(column);
            _stateNormalizer(normalized);
            Copy(normalized.data(), sigmaPoints.col(column).data(), NState);
        }

        Eigen::Matrix<float, NState, NState> processNoiseCovariance() const noexcept
        {
            return Symmetrize<NState>(_sqrtProcessNoise * _sqrtProcessNoise.transpose());
        }

        Eigen::Matrix<float, NState, NState> rebuildPredictedCovariance(const Eigen::Matrix<float, NState, kSigmaCount>& predictedSigma, const Eigen::Matrix<float, NState, 1>& predictedMean) const noexcept
        {
            Eigen::Matrix<float, NState, NState> covariance = processNoiseCovariance();
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                const Eigen::Matrix<float, NState, 1> residual = subtractAndNormalizeState(predictedSigma.col(column), predictedMean);
                covariance += _covarianceWeights(column) * (residual * residual.transpose());
            }
            return Symmetrize<NState>(covariance);
        }

        template <int M>
        Eigen::Matrix<float, M, M> rebuildInnovationCovariance(
            const Eigen::Matrix<float, M, kSigmaCount>& measurementSigma,
            const Eigen::Matrix<float, M, 1>& predictedMeasurement,
            const Eigen::Matrix<float, M, M>& sqrtMeasurementNoise) const noexcept
        {
            Eigen::Matrix<float, M, M> covariance =
                Symmetrize<M>(sqrtMeasurementNoise * sqrtMeasurementNoise.transpose());
            for (int column = 0; column < _activeSigmaCount; ++column)
            {
                const Eigen::Matrix<float, M, 1> residual =
                    subtractMeasurement<M>(measurementSigma.col(column), predictedMeasurement);
                covariance += _covarianceWeights(column) * (residual * residual.transpose());
            }
            return Symmetrize<M>(covariance);
        }

        Eigen::Matrix<float, NState, NState> rebuildPosteriorCovariance(const Eigen::Ref<const Eigen::Matrix<float, NState, Eigen::Dynamic>>& updateColumns) const noexcept
        {
            Eigen::Matrix<float, NState, NState> covariance = this->covariance();
            covariance -= updateColumns * updateColumns.transpose();
            return (covariance);
        }

        void makeSigmaPoints(
            const Eigen::Matrix<float, NState, 1>& mean,
            const Eigen::Matrix<float, NState, NState>& sqrtCovariance,
            float gamma,
            Eigen::Matrix<float, NState, kSigmaCount>& sigmaPoints) const noexcept
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

            Copy(mean.data(), sigmaPoints.col(0).data(), NState);
            normalizeSigmaColumn(sigmaPoints, 0);

            Eigen::Matrix<float, NState, 1> columnDelta;
            for (int column = 0; column < NState; ++column)
            {
                Scale(
                    sqrtCovariance.col(column).data(),
                    gamma,
                    columnDelta.data(),
                    NState);
                Add(
                    mean.data(),
                    columnDelta.data(),
                    sigmaPoints.col(column + 1).data(),
                    NState);
                Sub(
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
                _meanWeights(column) = MeanWeight(column);
                _covarianceWeights(column) = CovarianceWeight(column);
            }
        }

        void normalizeState(Eigen::Matrix<float, NState, 1>& state) const noexcept
        {
            if (_stateNormalizer != nullptr)
            {
                _stateNormalizer(state);
            }
        }

        Eigen::Matrix<float, NState, 1> _ownedState;
        Eigen::Matrix<float, NState, NState> _ownedSqrtCovariance;
        float _weightAlpha;
        float _weightBeta;
        float _weightKappa;
        Eigen::Matrix<float, NState, 1>& _state;
        Eigen::Matrix<float, NState, NState>& _sqrtCovariance;
        Eigen::Matrix<float, NState, NState> _sqrtProcessNoise;
        Eigen::Matrix<float, NState, kSigmaCount> _sigmaPoints;
        Eigen::Matrix<float, NState, 1> _sigmaMean;
        bool _predictionCacheValid;
        float _lastNis;
        void (*_stateNormalizer)(Eigen::Matrix<float, NState, 1>&);
        SigmaPointStrategy _sigmaPointStrategy;
        int _activeSigmaCount;
        Eigen::Matrix<float, kSigmaCount, 1> _meanWeights;
        Eigen::Matrix<float, kSigmaCount, 1> _covarianceWeights;
    };
}
