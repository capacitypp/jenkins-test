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

vector<MatrixXi*> RobustImageMatching::removeDuplicatePositions(const vector<MatrixXi*>& positionPtrs)
{
	vector<MatrixXi*> dst;
	for (unsigned i = 0; i < positionPtrs.size(); i++)
		if (!isDuplicatePosition(dst, *positionPtrs[i]))
			dst.push_back(positionPtrs[i]);
	return dst;
}

