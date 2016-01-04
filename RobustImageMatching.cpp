#include <iostream>

#include <Eigen/Dense>

#include "RobustImageMatching.h"
#include "MatrixUtil.h"
#include "EigenValue.h"

#define NEWTON_THRESHOLD	1.0e-10
#define NEWTON_MAXLOOP	100
#define RENORMALIZATION_THRESHOLD	1.0e-10
#define RENORMALIZATION_MAXLOOP	100

using namespace std;
using namespace Eigen;

bool RobustImageMatching::isDuplicatePosition(const MatrixXi& position1, const MatrixXi& position2)
{
	return (position1(0, 0) == position2(0, 0)) && (position1(1, 0) == position2(1, 0));
}

bool RobustImageMatching::isDuplicatePosition(const vector<MatrixXi*>& positionPtrs, const MatrixXi& position)
{
	for (unsigned i = 0; i < positionPtrs.size(); i++)
		if (isDuplicatePosition(*positionPtrs[i], position))
				return true;
	return false;
}

bool RobustImageMatching::isProtrudingPosition(const MatrixXi& position, int w, int width, int height)
{
	int x = position(0, 0);
	int y = position(1, 0);
	if (x - w / 2 < 0)
		return true;
	if (x + w / 2 >= width)
		return true;
	if (y - w / 2 < 0)
		return true;
	if (y + w / 2 >= height)
		return true;
	return false;
}

MatrixXd RobustImageMatching::computeT(const MatrixXd& gray, const MatrixXi& position, int w)
{
	MatrixXd T = gray.block(position(1, 0) - w /2, position(0, 0) - w / 2, w, w);
	T /= T.norm();
	return T;
};

double RobustImageMatching::computeJ(const MatrixXd& Tp, const MatrixXd& Tq)
{
	double norm = (Tp - Tq).norm();
	return norm * norm;
}

bool RobustImageMatching::searchCombination(const vector<CombinationPointer>& combinationPtrs, const CombinationPointer& combinationPtr)
{
	Combination* _ptr = combinationPtr.getPointer();
	int p = _ptr->getP();
	int q = _ptr->getQ();
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		Combination* ptr = combinationPtrs[i].getPointer();
		if (ptr->getP() == p || ptr->getQ() == q)
			return true;
	}
	return false;
}

void RobustImageMatching::evaluatePhiAndDiffPhi(const vector<double>& JsJBar, const vector<double>& Js, double* phi, double* diffPhi, double s)
{
	*phi = 0.0;
	*diffPhi = 0.0;
	for (unsigned i = 0; i < Js.size(); i++) {
		double J = Js[i];
		double JsJBare = JsJBar[i] * exp(-s * J);
		//cout << JsJBar[i] << endl;
		*phi += JsJBare;
		*diffPhi += -J * JsJBare;
	}
}

vector<MatrixXd> RobustImageMatching::computeFlow(const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXd*>& positionDoublePtrs1, const vector<MatrixXd*>& positionDoublePtrs2)
{
	vector<MatrixXd> rs;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		Combination* ptr = combinationPtr.getPointer();
		int p = ptr->getP();
		int q = ptr->getQ();
		const MatrixXd& positionDouble1 = *positionDoublePtrs1[p];
		const MatrixXd& positionDouble2 = *positionDoublePtrs2[q];
		rs.push_back(positionDouble2 - positionDouble1);
	}
	return rs;
}

MatrixXd RobustImageMatching::computeX(const MatrixXd& positionDouble, double f0)
{
	MatrixXd x(3, 1);
	x(0, 0) = positionDouble(0, 0) / f0;
	x(1, 0) = positionDouble(1, 0) / f0;
	x(2, 0) = 1.0;
	return x;
}

MatrixXd RobustImageMatching::computeTensorM(const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXd*>& xPtrs1, const vector<MatrixXd*>& xPtrs2, const vector<MatrixXd>& Ws)
{
	vector<MatrixXd> es;
	{
		MatrixXd e = MatrixXd::Zero(3, 1);
		e(0, 0) = 1.0;
		es.push_back(e);
		e = MatrixXd::Zero(3, 1);
		e(1, 0) = 1.0;
		es.push_back(e);
		e = MatrixXd::Zero(3, 1);
		e(2, 0) = 1.0;
		es.push_back(e);
	}
	vector<vector<MatrixXd>> exDashxtss;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		Combination* ptr = combinationPtr.getPointer();
		const MatrixXd& x1 = *xPtrs1[ptr->getP()];
		const MatrixXd& x2 = *xPtrs2[ptr->getQ()];
		MatrixXd xDashxt = x2 * x1.transpose();
		vector<MatrixXd> exDashxts;
		for (int j = 0; j < es.size(); j++) {
			const MatrixXd& e = es[j];
			MatrixXd exDashxt(3, 3);
			for (int k = 0; k < 3; k++)
				exDashxt.col(k) = CROSS_PRODUCT(e, xDashxt.col(k));
			exDashxts.push_back(exDashxt);
		}
		exDashxtss.push_back(exDashxts);
	}
	MatrixXd M = MatrixXd::Zero(9, 9);
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const MatrixXd& W = Ws[i];
		const vector<MatrixXd>& exDashxts = exDashxtss[i];
		for (int k = 0; k < 3; k++) {
			const MatrixXd& ekxDashxt = exDashxts[k];
			for (int l = 0; l < 3; l++) {
				const MatrixXd& elxDashxt = exDashxts[l];
				M += W(k, l) * tensorProduct(ekxDashxt, elxDashxt);
			}
		}
	}
	M /= combinationPtrs.size();
	return M;
}

MatrixXd RobustImageMatching::tensorProduct(const MatrixXd& a, const MatrixXd& b)
{
	MatrixXd c(a.rows() * a.cols(), b.rows() * b.cols());
	for (int i = 0; i < a.rows(); i++)
	for (int j = 0; j < a.cols(); j++)
	for (int k = 0; k < b.rows(); k++)
	for (int l = 0; l < b.cols(); l++)
		c(i * a.rows() + j, k * a.cols() + l) = a(i, j) * b(k, l);
	return c;
}

MatrixXd RobustImageMatching::computeTensorN(const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXd*>& xPtrs1, const vector<MatrixXd*>& xPtrs2, const vector<MatrixXd>& Ws, const vector<MatrixXd>& V0s)
{
	MatrixXd N = MatrixXd::Zero(9, 9);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				for (int l = 0; l < 3; l++) {
					for (unsigned alpha = 0; alpha < combinationPtrs.size(); alpha++) {
						const CombinationPointer& combinationPtr = combinationPtrs[alpha];
						Combination* ptr = combinationPtr.getPointer();
						const MatrixXd& x1 = *xPtrs1[ptr->getP()];
						const MatrixXd& x2 = *xPtrs2[ptr->getQ()];
						double xj1 = x1(j, 0);
						double xl1 = x1(l, 0);
						const MatrixXd& W = Ws[alpha];
						const MatrixXd& V0 = V0s[alpha];
						double V0jl = V0(j, l);
						for (int m = 0; m < 3; m++) {
							for (int n = 0; n < 3; n++) {
								double Wmn = W(m, n);
								for (int p = 0; p < 3; p++) {
									double imp = epsilon(i, m, p);
									double xp2 = x2(p, 0);
									for (int q = 0; q < 3; q++) {
										double knq = epsilon(k, n, q);
										double V0pq = V0(p, q);
										double xq2 = x2(q, 0);
										N(i * 3 + j, k * 3 + l) += imp * knq * Wmn * (V0jl * xp2 * xq2 + V0pq * xj1 * xl1);
									}
								}
							}
						}
					}
				}
			}
		}
	}
	N /= combinationPtrs.size();
	return N;
}

int RobustImageMatching::epsilon(int i, int j, int k)
{
	if (i == 0 && j == 1 && k == 2)
		return 1;
	if (i == 1 && j == 2 && k == 0)
		return 1;
	if (i == 2 && j == 0 && k == 1)
		return 1;
	if (i == 0 && j == 2 && k == 1)
		return -1;
	if (i == 2 && j == 1 && k == 0)
		return -1;
	if (i == 1 && j == 0 && k == 2)
		return -1;
	return 0;
}

vector<MatrixXi*> RobustImageMatching::removeDuplicatePositions(const vector<MatrixXi*>& positionPtrs)
{
	vector<MatrixXi*> dst;
	for (unsigned i = 0; i < positionPtrs.size(); i++)
		if (!isDuplicatePosition(dst, *positionPtrs[i]))
			dst.push_back(positionPtrs[i]);
	return dst;
}

vector<MatrixXi*> RobustImageMatching::removeProtrudingPositions(const vector<MatrixXi*>& positionPtrs, int w, int width, int height)
{
	vector<MatrixXi*> dst;
	for (unsigned i = 0; i < positionPtrs.size(); i++)
		if (!isProtrudingPosition(*positionPtrs[i], w, width, height))
			dst.push_back(positionPtrs[i]);
	return dst;
}

vector<MatrixXd> RobustImageMatching::computeTs(const MatrixXd& gray, const vector<MatrixXi*>& positionPtrs, int w)
{
	vector<MatrixXd> Ts;
	for (unsigned i = 0; i < positionPtrs.size(); i++)
		Ts.push_back(computeT(gray, *positionPtrs[i], w));
	return Ts;
}

vector<double> RobustImageMatching::computeJs(const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXd>& Tps, const vector<MatrixXd>& Tqs)
{
	vector<double> Js;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		Combination* ptr = combinationPtr.getPointer();
		int p = ptr->getP();
		int q = ptr->getQ();
		const MatrixXd& Tp = Tps[p];
		const MatrixXd& Tq = Tqs[q];
		Js.push_back(computeJ(Tp, Tq));
	}
	return Js;
}

void RobustImageMatching::setJs(vector<CombinationPointer>& combinationPtrs, const vector<double>& Js)
{
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		CombinationPointer& combinationPtr = combinationPtrs[i];
		double J = Js[i];
		combinationPtr.getValue() = J;
	}
}

vector<CombinationPointer> RobustImageMatching::one2OneReduction(const vector<CombinationPointer>& src)
{
	vector<CombinationPointer> combinationPtrs(src);
	sort(combinationPtrs.begin(), combinationPtrs.end());

	vector<CombinationPointer> dst;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		if (!searchCombination(dst, combinationPtr))
			dst.push_back(combinationPtr);
	}
	return dst;
}

double RobustImageMatching::computeJBar(const vector<double>& srcJs, unsigned L)
{
	vector<double> Js(srcJs);
	sort(Js.begin(), Js.end());
	double JBar = 0.0;
	for (unsigned i = 0; i < L; i++)
		JBar += Js[i];
	JBar /= L;
	return JBar;
}

double RobustImageMatching::solvePhi(const vector<double>& Js, double JBar) throw(NotConvergedException)
{
	vector<double> JsJBar;
	for (unsigned i = 0; i < Js.size(); i++)
		JsJBar.push_back(Js[i] - JBar);
	double s = 0.0;
	int loopCnt = 0;
	while (1) {
		double phi, diffPhi;
		evaluatePhiAndDiffPhi(JsJBar, Js, &phi, &diffPhi, s);
		if (fabs(phi) < NEWTON_THRESHOLD)
			break;
		if (++loopCnt >= NEWTON_MAXLOOP)
			throw NotConvergedException();
		s -= phi / diffPhi;
	}
	return s;
}

vector<double> RobustImageMatching::computeP0s(const vector<double>& Js, double s)
{
	vector<double> P0s;
	for (unsigned i = 0; i < Js.size(); i++)
		P0s.push_back(exp(-s * Js[i]));
	return P0s;
}

vector<CombinationPointer> RobustImageMatching::takeOutCombinations(const vector<CombinationPointer>& combinationPtrs, double threshold)
{
	vector<CombinationPointer> dst;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		if (combinationPtr.getValue() > threshold)
			dst.push_back(combinationPtr);
	}
	return dst;
}

vector<CombinationPointer> RobustImageMatching::getLocalCorrespondence(const vector<CombinationPointer>& srcCombinationPtrs, const vector<double>& P0s, double k)
{
	vector<CombinationPointer> combinationPtrs(srcCombinationPtrs);
	setJs(combinationPtrs, P0s);
	combinationPtrs = takeOutCombinations(combinationPtrs, exp(-k * k / 2));
	return one2OneReduction(combinationPtrs);
}

vector<double> RobustImageMatching::computeP1s(const vector<CombinationPointer>& combinationPtrs, const vector<CombinationPointer>& localCorrespondence, const vector<MatrixXd*>& positionDoublePtrs1, const vector<MatrixXd*>& positionDoublePtrs2) throw(InvalidDataNumException, InvalidDeterminantException)
{
	unsigned n0 = localCorrespondence.size();
	if (n0 == 0)
		throw InvalidDataNumException();
	double Z = 0.0;
	for (unsigned i = 0; i < n0; i++)
		Z += localCorrespondence[i].getValue();

	vector<MatrixXd> rus = computeFlow(localCorrespondence, positionDoublePtrs1, positionDoublePtrs2);

	MatrixXd rm = MatrixXd::Zero(2, 1);
	for (unsigned i = 0; i < n0; i++) {
		const MatrixXd& ru = rus[i];
		double P0 = localCorrespondence[i].getValue();
		rm += P0 / Z * ru;
	}

	MatrixXd V = MatrixXd::Zero(2, 2);
	for (unsigned i = 0; i < n0; i++) {
		const MatrixXd& ru = rus[i];
		double P0 = localCorrespondence[i].getValue();
		MatrixXd rurm = ru - rm;
		V += P0 / Z * rurm * rurm.transpose();
	}

	if (V.determinant() == 0.0)
		throw InvalidDeterminantException();
	MatrixXd VInverse = V.inverse();
	vector<MatrixXd> rLambdas = computeFlow(combinationPtrs, positionDoublePtrs1, positionDoublePtrs2);

	vector<double> P1s;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		const MatrixXd& rLambda = rLambdas[i];
		MatrixXd rLambdarm = rLambda - rm;
		P1s.push_back(exp(-DOT_PRODUCT(rLambdarm, VInverse * rLambdarm)));
	}

	return P1s;
}

vector<CombinationPointer> RobustImageMatching::getSpatialCorrespondence(const vector<CombinationPointer>& srcCombinationPtrs, const vector<double>& P0s, const vector<double>& P1s, double k)
{
	vector<CombinationPointer> combinationPtrs(srcCombinationPtrs);
	vector<double> P0P1s;
	for (unsigned i = 0; i < P0s.size(); i++)
		P0P1s.push_back(P0s[i] * P1s[i]);
	setJs(combinationPtrs, P0P1s);
	combinationPtrs = takeOutCombinations(combinationPtrs, exp(-2 * k * k / 2));
	return one2OneReduction(combinationPtrs);
}

vector<MatrixXd> RobustImageMatching::computeXs(const vector<MatrixXd*>& positionDoublePtrs, double f0)
{
	vector<MatrixXd> xs;
	for (unsigned i = 0; i < positionDoublePtrs.size(); i++)
		xs.push_back(computeX(*positionDoublePtrs[i], f0));
	return xs;
}

MatrixXd RobustImageMatching::computeH(const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXd*> xPtrs1, const vector<MatrixXd*> xPtrs2) throw(EigenValueException, NotConvergedException)
{
	double c = 0.0;
	vector<MatrixXd> Ws;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		Ws.push_back(MatrixXd::Identity(3, 3));
	}
	vector<MatrixXd> V0s;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		MatrixXd V0 = MatrixXd::Identity(3, 3);
		V0(2, 2) = 0.0;
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		V0s.push_back(V0 / combinationPtr.getValue());
	}
	MatrixXd H933(3, 3);
	int loopCnt = 0;
	while (1) {
		MatrixXd M = computeTensorM(combinationPtrs, xPtrs1, xPtrs2, Ws);
		MatrixXd N = computeTensorN(combinationPtrs, xPtrs1, xPtrs2, Ws, V0s);
		MatrixXd MHat = M - c * N;
		vector<EigenValue> eigenvalues;
		try {
			eigenvalues = EigenValue::solve(MHat);
		}
		catch (const EigenValueException& e) {
			throw e;
		}
		const EigenValue& eigenvalue = eigenvalues[0];
		double lambda9 = eigenvalue.getEigenvalue();
		MatrixXd H9 = eigenvalue.getEigenvector();
		H9 /= H9.norm();
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			H933(i, j) = H9(i * 3 + j, 0);
		if (fabs(lambda9) < RENORMALIZATION_THRESHOLD)
			break;
		if (++loopCnt >= RENORMALIZATION_MAXLOOP)
			throw NotConvergedException();
		c += lambda9 / DOT_PRODUCT(H9, N * H9);
		for (unsigned i = 0; i < Ws.size(); i++) {
			MatrixXd& W = Ws[i];
			const CombinationPointer& combinationPtr = combinationPtrs[i];
			Combination* ptr = combinationPtr.getPointer();
			const MatrixXd& x1 = *xPtrs1[ptr->getP()];
			const MatrixXd& x2 = *xPtrs2[ptr->getQ()];
			const MatrixXd& V0 = V0s[i];
			MatrixXd left = H933 * V0 * H933.transpose();
			for (int j = 0; j < left.cols(); j++)
				left.col(j) = CROSS_PRODUCT(x2, left.col(j));
			for (int j = 0; j < left.cols(); j++)
				left.col(j) = CROSS_PRODUCT(left.col(j), x2);
			MatrixXd H933x1 = H933 * x1;
			MatrixXd right(3, 3);
			for (int j = 0; j < right.cols(); j++)
				right.col(j) = CROSS_PRODUCT(H933x1, V0.col(j));
			for (int j = 0; j < right.cols(); j++)
				right.col(j) = CROSS_PRODUCT(right.col(j), H933x1);
			MatrixXd m = left + right;
			JacobiSVD<MatrixXd> svd(m, ComputeFullU | ComputeFullV);
			MatrixXd U, S, V;
			S = svd.singularValues();
			U = svd.matrixU();
			V = svd.matrixV();
			for (int j = 0; j < S.rows() - 1; j++)
				S(j, 0) = 1 / S(j, 0);
			S(S.rows() - 1, 0) = 0.0;
			W = V * S.asDiagonal() * U.transpose();
		}
	}
	return H933;
}

vector<double> RobustImageMatching::computeDs(const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXd*>& xPtrs1, const vector<MatrixXd*>& xPtrs2, const MatrixXd& H)
{
	vector<double> Ds;
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		Combination* ptr = combinationPtr.getPointer();
		const MatrixXd& x1 = *xPtrs1[ptr->getP()];
		const MatrixXd& x2 = *xPtrs2[ptr->getQ()];
		MatrixXd Hx1 = H * x1;
		Hx1 /= Hx1(2, 0);
		double tmp = (x2 - Hx1).norm();
		Ds.push_back(tmp * tmp);
	}
	return Ds;
}

vector<CombinationPointer> RobustImageMatching::getGlobalCorrespondence(const vector<CombinationPointer>& srcCombinationPtrs, const vector<double>& P0s, const vector<double>& P1s, const vector<double>& P2s, double k)
{
	vector<CombinationPointer> combinationPtrs(srcCombinationPtrs);
	vector<double> P0P1P2s;
	for (unsigned i = 0; i < P0s.size(); i++)
		P0P1P2s.push_back(P0s[i] * P1s[i] * P2s[i]);
	setJs(combinationPtrs, P0P1P2s);
	combinationPtrs = takeOutCombinations(combinationPtrs, exp(-3 * k * k / 2));
	return one2OneReduction(combinationPtrs);
}

