#ifndef ___Class_MatrixConverter
#define ___Class_MatrixConverter

#include <vector>

#include <Eigen/Core>

class MatrixConverter {
	static Eigen::MatrixXd convert2MatrixXd(const Eigen::MatrixXi& src);
public:
	static std::vector<Eigen::MatrixXd> convert2MatrixXd(const std::vector<Eigen::MatrixXi>& src);
	static std::vector<Eigen::MatrixXi*> convert2MatrixPointer(std::vector<Eigen::MatrixXi>& src);
};

#endif

