#ifndef ___Class_Combination
#define ___Class_Combination

#include <vector>

class Combination {
	int p, q;
public:
	Combination(int _p, int _q) : p(_p), q(_q) {}
	inline int getP(void) const { return p; }
	inline int getQ(void) const { return q; }
	inline int& getP(void) { return p; }
	inline int& getQ(void) { return q; }
public:
	static std::vector<Combination> generateCombinations(int pSize, int qSize);
};

class CombinationPointer {
	Combination* ptr;
	double value;
public:
	CombinationPointer(Combination* _ptr) : ptr(_ptr) { }
	inline Combination* getPointer(void) const { return ptr; }
	inline double getValue(void) const { return value; }
	inline double& getValue(void) { return value; }
private:
	static CombinationPointer convert2CombinationPointer(Combination& combination);
public:
	static std::vector<CombinationPointer> convert2CombinationPointer(std::vector<Combination>& combinations);
};

#endif

