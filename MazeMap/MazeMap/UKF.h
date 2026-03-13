#pragma once
#include "..\eigen-5.0.0\Eigen\Eigen"
#include "..\eigen-5.0.0\Eigen\Dense"
#include "..\eigen-5.0.0\Eigen\Cholesky"
namespace MazeMap {
	constexpr float ce_sqrt(float val)
	{
		if (val < 0) return -1.0; // Or handle error
		double root = val / 2.0;
		for (int i = 0; i < 10; ++i) { // 10 iterations are sufficient for double precision
			root = 0.5 * (root + val / root);
		}
		return root;
	}
	template <int N_STATE, int M_MEAS, int P_CONT>
	class UKF
	{

	public:
		typedef Eigen::Vector<float, N_STATE> StateVec;
		typedef Eigen::Matrix<float, N_STATE, N_STATE> StateMat;
		typedef Eigen::Vector<float, M_MEAS> MeasVec;
		typedef Eigen::Matrix<float, M_MEAS, M_MEAS> MeasMat;
		typedef Eigen::Matrix<float, N_STATE, 2 * N_STATE + 1> SigmaMat;
		typedef Eigen::Matrix<float, N_STATE, M_MEAS> SMMat;
		typedef Eigen::Vector<float, P_CONT> ControlVec;
		typedef Eigen::Matrix<float, M_MEAS, 2 * N_STATE + 1> SigmaMMat;
		StateVec(*ProcessFunction)(StateVec state, ControlVec control);
		MeasVec(*MeasFunction)(StateVec state);

		UKF() : _P(), _Ppred(), _Q(), _R(),_u(),
			_X(), _Xp(), _xpred(),
			_z(), _Zp(), _zpred(), stateMat_var1(), _K()
		{
			ldlt = Eigen::LLT<StateMat>(_P);
			w_m = Eigen::Vector<float, 2 * N_STATE + 1>();
			w_c = Eigen::Vector<float, 2 * N_STATE + 1>();
			w_m[0] = lambda / (N_STATE + lambda);
			w_c[0] = w_m[0] + (1 - alpha * alpha + _beta);
			for (size_t i = 1; i < 2*N_STATE; i++)
			{
				w_m[i] = 0.5f / (N_STATE + lambda);
				w_c[i] = w_m[i];
			}
		}

		const StateVec& state() const { return _X; }
		const StateMat& covariance() const { return _P; }

		// -----------------------------
		// Setters / getters
		// -----------------------------
		void setState(const StateVec& x0, const StateMat& P0) {
			_X.noalias() = x0;
			_P.noalias() = P0;
			_symmetrize(_P);
		}

		void setNoise(const StateMat& Q, const MeasMat& R) {
			_Q.noalias() = Q;
			_R.noalias() = R;
			_symmetrize(_Q);
			_symmetrize(_R);
		}

		void MakeSigmas(SigmaMat& sigmaPoints)
		{
			auto ldltP = robustCholesky(_P);
			stateMat_var1 = ldlt.matrixL();
			sigmaPoints.col(0) = _X;
			for (size_t i = 0; i < N_STATE; i++)
			{
				sigmaPoints.col(i+1) = _X + gamma * stateMat_var1.col(i);
				sigmaPoints.col(i + 1 + N_STATE) = _X - gamma * stateMat_var1.col(i);
			}
		}

		void Predict(ControlVec cont)
		{
			MakeSigmas(_Xp);
			for (size_t i = 0; i < 2*N_STATE+1; i++)
			{
				_Xp.col(i) =  ProcessFunction(_Xp.col(i), _u);
			}
			_xpred.noalias() = _Xp.col(0) * w_m[0];
			for (size_t i = 0; i < 2 * N_STATE + 1; i++)
			{
				_xpred.noalias() += w_m[i] * _Xp.col(i);
			}
			
			StateVec dx = _Xp.col(0) - _xpred;
			_Ppred.noalias() = w_c(0) * (dx * dx.transpose());
			for (size_t i = 0; i < 2 * N_STATE + 1; i++)
			{
				dx = _Xp.col(i) - _xpred;
				_Ppred.noalias() += w_c[0]*(dx * dx.transpose());
			}
			_Ppred.noalias() += _Q;
			_symmetrize(_Ppred);

			_X.noalias() = _xpred;
			_P.noalias() = _Ppred;
		}
		void Update(const MeasVec& z)
		{
			for (size_t i = 0; i < 2 * N_STATE + 1; ++i)
			{
				_Zp.col(i) = MeasFunction(_Xp.col(i));
			}
			_zpred.noalias() = _Zp.col(0) * w_m[0];
			for (size_t i = 0; i < 2 * N_STATE + 1; i++)
			{
				_zpred.noalias() += w_m[i] * _Zp.col(i);
			}

			MeasVec dz = _Zp.col(0) - _zpred;
			StateVec dx = _Xp.col(0) - _X;

			_Szz.noalias() = w_c[0] * (dz * dz.transpose());
			_Pxz.noalias() = w_c[0] * (dx * dz.transpose());
			for (size_t i = 0; i < 2 * N_STATE + 1; i++)
			{
				dz = _Zp.col(i) - _zpred;
				dx = _Xp.col(i) - _X;

				_Szz.noalias() = w_c[1] * (dz * dz.transpose());
				_Pxz.noalias() = w_c[1] * (dx * dz.transpose());
			}
			_Szz.noalias() += _R;
			_symmetrize(_Szz);
			Eigen::LLT<MeasMat> ldlt2 = robustCholesky(_Szz);
			const Eigen::Matrix<float, M_MEAS, N_STATE> Y = ldlt2.solve(_Pxz.transpose());
			_K = Y.transpose();

			const MeasVec r = z - _zpred;
			_X.noalias() += _K * r;
			_P.noalias() -= _K * _Szz * _K.transpose();
		}

	private:
		static Eigen::LLT<MeasMat> robustCholesky(const MeasMat& A) {
			MeasMat M = A;
			_symmetrize(M);

			Eigen::LLT<MeasMat> llt;
			llt.compute(M);
			if (llt.info() == Eigen::Success) return llt;

			float jitter = 1e-12f;
			float diag_scale = M.diagonal().cwiseAbs().maxCoeff();
			if (diag_scale < 1.0f)
			{
				diag_scale = 1.0f;
			}

			for (int k = 0; k < 8; ++k)
			{
				MeasMat J = M;
				J.diagonal().array() += jitter * diag_scale;
				llt.compute(J);
				if (llt.info() == Eigen::Success)
				{
					return llt;
				}
				jitter *= 10.0f;
			}
		}
		static Eigen::LLT<StateMat> robustCholesky(const StateMat& A) {
			StateMat M = A;
			_symmetrize(M);

			Eigen::LLT<StateMat> llt;
			llt.compute(M);
			if (llt.info() == Eigen::Success) return llt;

			float jitter = 1e-12f;
			float diag_scale = M.diagonal().cwiseAbs().maxCoeff();
			if (diag_scale < 1.0f)
			{
				diag_scale = 1.0f;
			}
			for (int k = 0; k < 8; ++k)
			{
				StateMat J = M;
				J.diagonal().array() += jitter * diag_scale;
				llt.compute(J);
				if (llt.info() == Eigen::Success)
				{
					return llt;
				}
				jitter *= 10.0f;
			}
		}
		static constexpr float alpha = 0.001;
		static constexpr float kappa = 3-N_STATE;
		static constexpr float lambda = alpha*alpha*(N_STATE + kappa)-N_STATE;
		static constexpr float gamma = ce_sqrt(N_STATE+lambda);
		static constexpr float _beta = 2;
		Eigen::Vector<float, 2 * N_STATE + 1> w_m;
		Eigen::Vector<float, 2 * N_STATE + 1> w_c;
		static void _symmetrize(MeasMat& matrix)
		{
			MeasMat tmp = matrix;
			tmp += matrix.transpose();
			matrix.noalias() = 0.5f * tmp;
		}
		static void _symmetrize(StateMat& matrix)
		{
			StateMat tmp = matrix;
			tmp += matrix.transpose();
			matrix.noalias() = 0.5f * tmp;
		}
		Eigen::LLT<StateMat> ldlt;
		// State Covariance
		StateMat _P;
		// State Covariance
		StateMat _Ppred;
		// Additive Covariance
		StateMat _Q;
		MeasMat _R;
		// State
		StateVec _X;
		// Control
		ControlVec _u;
		// Measurement
		MeasVec _z;
		// Sigmas
		SigmaMat _Xp;
		StateVec _xpred;
		// WorkingSpace
		StateMat stateMat_var1;
		MeasMat _Szz;
		SMMat _Pxz;
		SigmaMMat _Zp;
		// WorkingSpace
		SigmaMat sigmam_var1;
		MeasVec  _zpred;
		SMMat  _K;      // Kalman gain
	};
}
