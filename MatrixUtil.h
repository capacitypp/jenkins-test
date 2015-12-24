#ifndef ___Class_MatrixUtil
#define ___Class_MatrixUtil

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

class MatrixUtil {
public:
	static Eigen::MatrixXd convertGray2MatrixXd(const cv::Mat& gray);
};

#endif

