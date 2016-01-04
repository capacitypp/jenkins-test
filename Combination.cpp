#include <iostream>
#include <fstream>

#include "Combination.h"

using namespace std;
using namespace Eigen;

vector<Combination> Combination::generateCombinations(int pSize, int qSize)
{
	vector<Combination> combinations;
	for (int p = 0; p < pSize; p++)
	for (int q = 0; q < qSize; q++)
		combinations.push_back(Combination(p, q));
	return combinations;
}

CombinationPointer CombinationPointer::convert2CombinationPointer(Combination& combination)
{
	return CombinationPointer(&combination);
}

vector<CombinationPointer> CombinationPointer::convert2CombinationPointer(vector<Combination>& combinations)
{
	vector<CombinationPointer> dst;
	for (unsigned i = 0; i < combinations.size(); i++)
		dst.push_back(convert2CombinationPointer(combinations[i]));
	return dst;
}

void CombinationPointer::write(const string& fpath, const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXi*>& positionPtrs1, const vector<MatrixXi*>& positionPtrs2)
{
	ofstream file(fpath.c_str());
	unsigned n = combinationPtrs.size();
	file << n << endl;
	for (unsigned i = 0; i < n; i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		Combination* ptr = combinationPtr.getPointer();
		const MatrixXi& position1 = *positionPtrs1[ptr->getP()];
		const MatrixXi& position2 = *positionPtrs2[ptr->getQ()];
		file << "(";
		file << position1(0, 0) << "," << position1(1, 0);
		file << ")--(";
		file << position2(0, 0) << "," << position2(1, 0);
		file << ")" << endl;
	}
}

