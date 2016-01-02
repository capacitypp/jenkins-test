#include "Combination.h"

using namespace std;

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

