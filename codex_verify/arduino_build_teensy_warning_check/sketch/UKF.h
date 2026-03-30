#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\UKF.h"
#pragma once

#include "MazeMapEigen.h"

#include <cmath>
#include <limits>

namespace MazeMap
{
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
            return (spread > 0.0f) ? std::sqrt(spread) : 0.0f;
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

    template <int NState>
    struct SrUkfMath
    {
        using Vector = Eigen::Matrix<float, NState, 1>;
        using Matrix = Eigen::Matrix<float, NState, NState>;

        template <int Cols>
        using MatrixNxCols = Eigen::Matrix<float, NState, Cols>;

        static Matrix Symmetrize(const Matrix& input) noexcept
        {
            return 0.5f * (input + input.transpose());
        }

        static bool FactorCovariance(const Matrix& covariance, Matrix& sqrtCovariance) noexcept
        {
            Matrix candidate = Symmetrize(covariance);
            Eigen::LLT<Matrix> llt;
            llt.compute(candidate);
            if (llt.info() == Eigen::Success)
            {
                sqrtCovariance = llt.matrixL();
                return true;
            }

            float jitter = 1.0e-9f;
            const float scale = (std::max)(1.0f, candidate.diagonal().cwiseAbs().maxCoeff());
            for (int attempt = 0; attempt < 8; ++attempt)
            {
                Matrix regularized = candidate;
                regularized.diagonal().array() += jitter * scale;
                llt.compute(regularized);
                if (llt.info() == Eigen::Success)
                {
                    sqrtCovariance = llt.matrixL();
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
            vector *= std::sqrt(std::fabs(weight));

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

                const float radial = std::sqrt(radialSquared);
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
                    vector(row) = (c * vector(row)) - (s * previous);
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
            return whitened.squaredNorm();
        }

        template <int M>
        static float ComputeNis(
            const Eigen::Matrix<float, M, 1>& innovation,
            const Eigen::Matrix<float, M, M>& innovationSqrt) noexcept
        {
            const Eigen::Matrix<float, M, 1> whitened =
                innovationSqrt.template triangularView<Eigen::Lower>().solve(innovation);
            return whitened.squaredNorm();
        }
    };

    template <int NState, int MDefault, int NControl>
    class UKF
    {
    public:
        static constexpr int kSigmaCount = (2 * NState) + 1;

        using StateVec = Eigen::Matrix<float, NState, 1>;
        using StateMat = Eigen::Matrix<float, NState, NState>;
        using ControlVec = Eigen::Matrix<float, NControl, 1>;
        using DefaultMeasVec = Eigen::Matrix<float, MDefault, 1>;
        using DefaultMeasMat = Eigen::Matrix<float, MDefault, MDefault>;
        using SigmaMat = Eigen::Matrix<float, NState, kSigmaCount>;

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
        {
        }

        const StateVec& state() const noexcept { return _state; }
        const StateMat& sqrtCovariance() const noexcept { return _sqrtCovariance; }
        StateMat covariance() const noexcept { return _sqrtCovariance * _sqrtCovariance.transpose(); }
        float lastNis() const noexcept { return _lastNis; }

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
            _predictionCacheValid = false;
        }

        template <typename ProcessFn>
        bool Predict(float dt, const ControlVec& control, ProcessFn&& processFn) noexcept
        {
            const SigmaMat priorSigma = makeSigmaPoints(_state, _sqrtCovariance);
            SigmaMat predictedSigma = SigmaMat::Zero();
            for (int column = 0; column < kSigmaCount; ++column)
            {
                predictedSigma.col(column) = processFn(priorSigma.col(column), control, dt);
                StateVec normalizedSigma = predictedSigma.col(column);
                normalizeState(normalizedSigma);
                predictedSigma.col(column) = normalizedSigma;
            }

            StateVec predictedMean = StateVec::Zero();
            for (int column = 0; column < kSigmaCount; ++column)
            {
                predictedMean += _weights.meanWeight(column) * predictedSigma.col(column);
            }
            normalizeState(predictedMean);

            Eigen::Matrix<float, NState, (2 * NState) + NState> stacked;
            for (int column = 1; column < kSigmaCount; ++column)
            {
                StateVec residual = predictedSigma.col(column) - predictedMean;
                normalizeState(residual);
                stacked.col(column - 1) = std::sqrt(_weights.covarianceWeight(column)) * residual;
            }
            stacked.template rightCols<NState>() = _sqrtProcessNoise;

            StateMat predictedSqrt = SrUkfMath<NState>::template QrSquareRoot<(2 * NState) + NState>(stacked);
            StateVec centralResidual = predictedSigma.col(0) - predictedMean;
            normalizeState(centralResidual);
            if (!SrUkfMath<NState>::CholUpdate(predictedSqrt, centralResidual, _weights.covarianceWeight(0)))
            {
                return false;
            }

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
            if (!_predictionCacheValid)
            {
                _sigmaPoints = makeSigmaPoints(_state, _sqrtCovariance);
                _sigmaMean = _state;
            }

            Eigen::Matrix<float, M, kSigmaCount> measurementSigma = Eigen::Matrix<float, M, kSigmaCount>::Zero();
            for (int column = 0; column < kSigmaCount; ++column)
            {
                measurementSigma.col(column) = measurementFn(_sigmaPoints.col(column));
            }

            Eigen::Matrix<float, M, 1> predictedMeasurement = Eigen::Matrix<float, M, 1>::Zero();
            for (int column = 0; column < kSigmaCount; ++column)
            {
                predictedMeasurement += _weights.meanWeight(column) * measurementSigma.col(column);
            }

            Eigen::Matrix<float, M, (2 * NState) + M> stacked;
            for (int column = 1; column < kSigmaCount; ++column)
            {
                const Eigen::Matrix<float, M, 1> residual = measurementSigma.col(column) - predictedMeasurement;
                stacked.col(column - 1) = std::sqrt(_weights.covarianceWeight(column)) * residual;
            }
            stacked.template rightCols<M>() = sqrtMeasurementNoise;

            using MeasMat = Eigen::Matrix<float, M, M>;
            MeasMat innovationSqrt =
                SrUkfMath<M>::template QrSquareRoot<(2 * NState) + M>(stacked);
            const Eigen::Matrix<float, M, 1> centralMeasurementResidual = measurementSigma.col(0) - predictedMeasurement;
            if (!SrUkfMath<M>::CholUpdate(innovationSqrt, centralMeasurementResidual, _weights.covarianceWeight(0)))
            {
                return false;
            }

            Eigen::Matrix<float, NState, M> crossCovariance = Eigen::Matrix<float, NState, M>::Zero();
            for (int column = 0; column < kSigmaCount; ++column)
            {
                StateVec stateResidual = _sigmaPoints.col(column) - _sigmaMean;
                normalizeState(stateResidual);
                const Eigen::Matrix<float, M, 1> measurementResidual = measurementSigma.col(column) - predictedMeasurement;
                crossCovariance += _weights.covarianceWeight(column) * (stateResidual * measurementResidual.transpose());
            }

            const Eigen::Matrix<float, M, NState> solvedLower =
                innovationSqrt.template triangularView<Eigen::Lower>().solve(crossCovariance.transpose());
            const Eigen::Matrix<float, M, NState> solvedUpper =
                innovationSqrt.transpose().template triangularView<Eigen::Upper>().solve(solvedLower);
            const Eigen::Matrix<float, NState, M> kalmanGain = solvedUpper.transpose();

            const Eigen::Matrix<float, M, 1> innovation = measurement - predictedMeasurement;
            _lastNis = SrUkfMath<M>::ComputeNis(innovation, innovationSqrt);
            if (std::isfinite(nisThreshold) && (nisThreshold > 0.0f) && (_lastNis >= nisThreshold))
            {
                return false;
            }

            _state += kalmanGain * innovation;
            normalizeState(_state);

            StateMat updatedSqrt = _sqrtCovariance;
            const Eigen::Matrix<float, NState, M> updateColumns = kalmanGain * innovationSqrt;
            for (int column = 0; column < M; ++column)
            {
                if (!SrUkfMath<NState>::CholUpdate(updatedSqrt, updateColumns.col(column), -1.0f))
                {
                    return false;
                }
            }

            _sqrtCovariance = updatedSqrt;
            _predictionCacheValid = false;
            return true;
        }

    private:
        SigmaMat makeSigmaPoints(const StateVec& mean, const StateMat& sqrtCovariance) const noexcept
        {
            SigmaMat sigmaPoints = SigmaMat::Zero();
            sigmaPoints.col(0) = mean;
            const float gamma = _weights.gamma();
            for (int column = 0; column < NState; ++column)
            {
                sigmaPoints.col(column + 1) = mean + (gamma * sqrtCovariance.col(column));
                sigmaPoints.col(column + 1 + NState) = mean - (gamma * sqrtCovariance.col(column));
                StateVec plusSigma = sigmaPoints.col(column + 1);
                StateVec minusSigma = sigmaPoints.col(column + 1 + NState);
                normalizeState(plusSigma);
                normalizeState(minusSigma);
                sigmaPoints.col(column + 1) = plusSigma;
                sigmaPoints.col(column + 1 + NState) = minusSigma;
            }
            return sigmaPoints;
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
    };
}
