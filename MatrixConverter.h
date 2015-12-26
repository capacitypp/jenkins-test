#ifndef ___Class_MatrixConverter
#define ___Class_MatrixConverter

#include <vector>

#include <Eigen/Core>

class MatrixConverter {
public:
	static Eigen::MatrixXd convert2MatrixXd(const Eigen::MatrixXi& src);
	static std::vector<Eigen::MatrixXd> convert2MatrixXd(const std::vector<Eigen::MatrixXi>& src);
};

#endif

