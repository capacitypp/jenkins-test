#ifndef ___Class_MatrixUtil
#define ___Class_MatrixUtil

#include <string>

#include <opencv2/core.hpp>
#include <Eigen/Core>

inline double DOT_PRODUCT(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
	double ret = 0.0;
	for (int i = 0; i < a.rows(); i++)
		ret += a(i, 0) * b(i, 0);
	return ret;
}

inline Eigen::MatrixXd CROSS_PRODUCT(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
{
	Eigen::MatrixXd c(3, 1);
	c(0, 0) = a(1, 0) * b(2, 0) - a(2, 0) * b(1, 0);
	c(1, 0) = a(2, 0) * b(0, 0) - a(0, 0) * b(2, 0);
	c(2, 0) = a(0, 0) * b(1, 0) - a(1, 0) * b(0, 0);
	return c;
}

class MatrixUtil {
public:
	static Eigen::MatrixXd convertGray2MatrixXd(const cv::Mat& gray);
};

#endif

