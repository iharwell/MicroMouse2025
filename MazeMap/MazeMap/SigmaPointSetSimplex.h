#pragma once

#include "Defines.h"
#include "EigenCompat.h"

#include <cmath>

namespace MazeMap
{
    class EXPORT SigmaPointSetSimplex
    {
    public:
        static constexpr int ActiveSigmaCountForDimension(int stateDimension) noexcept
        {
            return (stateDimension > 0) ? (stateDimension + 2) : 0;
        }

        template <int NState, typename MeanWeightVector, typename CovarianceWeightVector>
        static void ComputeWeights(
            MeanWeightVector& meanWeights,
            CovarianceWeightVector& covarianceWeights) noexcept
        {
            static_assert(NState > 0, "SigmaPointSetSimplex requires a positive state dimension.");

            meanWeights.setZero();
            covarianceWeights.setZero();

            constexpr int kSigmaCount = NState + 2;
            const float outerWeight = 1.0f / static_cast<float>(NState + 1);
            meanWeights(0) = 0.0f;
            covarianceWeights(0) = 2.0f;
            for (int column = 1; column < kSigmaCount; ++column)
            {
                meanWeights(column) = outerWeight;
                covarianceWeights(column) = outerWeight;
            }
        }

        template <int NState, typename SigmaMatrix>
        static bool GenerateSigmaPoints(
            const Eigen::Matrix<float, NState, 1>& mean,
            const Eigen::Matrix<float, NState, NState>& sqrtCovariance,
            SigmaMatrix& sigmaPoints) noexcept
        {
            static_assert(NState > 0, "SigmaPointSetSimplex requires a positive state dimension.");

            constexpr int kSigmaCount = NState + 2;
            if (!mean.allFinite() || !sqrtCovariance.allFinite() || (sigmaPoints.cols() < kSigmaCount))
            {
                return false;
            }

            sigmaPoints.setZero();
            sigmaPoints.col(0) = mean;

            const float outerWeight = 1.0f / static_cast<float>(NState + 1);
            if (!(outerWeight > 0.0f) || !std::isfinite(outerWeight))
            {
                return false;
            }

            Eigen::Matrix<float, NState, kSigmaCount> canonicalSimplex =
                Eigen::Matrix<float, NState, kSigmaCount>::Zero();
            if constexpr (NState >= 1)
            {
                const float firstAxisScale = 1.0f / std::sqrt(2.0f * outerWeight);
                canonicalSimplex(0, 1) = -firstAxisScale;
                canonicalSimplex(0, 2) = firstAxisScale;

                for (int dimension = 2; dimension <= NState; ++dimension)
                {
                    const int row = dimension - 1;
                    const float appendedScale =
                        std::sqrt(1.0f / (static_cast<float>(dimension) *
                            static_cast<float>(dimension + 1) * outerWeight));
                    const float terminalScale =
                        std::sqrt(static_cast<float>(dimension) /
                            (static_cast<float>(dimension + 1) * outerWeight));

                    for (int column = 1; column <= dimension; ++column)
                    {
                        canonicalSimplex(row, column) = -appendedScale;
                    }
                    canonicalSimplex(row, dimension + 1) = terminalScale;
                }
            }

            for (int vertexIndex = 1; vertexIndex < kSigmaCount; ++vertexIndex)
            {
                sigmaPoints.col(vertexIndex) =
                    mean + (sqrtCovariance * canonicalSimplex.col(vertexIndex));
            }

            return sigmaPoints.leftCols(kSigmaCount).allFinite();
        }
    };
}
