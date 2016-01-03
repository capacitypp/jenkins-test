#include <iostream>

#include <Eigen/Dense>

#include "RobustImageMatching.h"
#include "MatrixUtil.h"

#define NEWTON_THRESHOLD	1.0e-10
#define NEWTON_MAXLOOP	1000

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

