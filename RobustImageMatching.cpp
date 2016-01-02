#include "RobustImageMatching.h"

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

bool RobustImageMatching::isProtrudingPosition(const Eigen::MatrixXi& position, int w, int width, int height)
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

bool RobustImageMatching::searchCombination(const std::vector<CombinationPointer>& combinationPtrs, const CombinationPointer& combinationPtr)
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
		Js.push_back(computeJ(Tq, Tq));
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

