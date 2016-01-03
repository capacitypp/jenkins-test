#ifndef ___Class_EigenValue
#define ___Class_EigenValue

#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "Exception.h"

class EigenValue {
	double eigenvalue;
	Eigen::MatrixXd eigenvector;
public:
	EigenValue(double value, const Eigen::MatrixXd& vector) : eigenvalue(value), eigenvector(vector) {}
	double getEigenvalue(void) const { return eigenvalue; }
	double& getEigenvalue(void) { return eigenvalue; }
	Eigen::MatrixXd getEigenvector(void) const { return eigenvector; }
	Eigen::MatrixXd& getEigenvector(void) { return eigenvector; }
	inline bool operator<(const EigenValue& e) { return eigenvalue < e.eigenvalue; }
	static std::vector<EigenValue> solve(const Eigen::MatrixXd& a) throw(EigenValueException);
	static std::vector<EigenValue> solve(const Eigen::MatrixXd& a, const Eigen::MatrixXd& g) throw(EigenValueException);
};

#endif

