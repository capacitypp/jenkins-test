#include <iostream>
#include <fstream>

#include "PositionUtil.h"

using namespace std;
using namespace Eigen;

vector<MatrixXi> PositionUtil::readPositions(const string& fpath)
{
	ifstream file(fpath.c_str());

	unsigned n;
	file >> n;
	vector<MatrixXi> positions;
	for (unsigned i = 0; i < n; i++) {
		MatrixXi position(2, 1);
		for (int j = 0; j < position.rows(); j++)
			file >> position(j, 0);
		positions.push_back(position);
	}

	return positions;
}

