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

