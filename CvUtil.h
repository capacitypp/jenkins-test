#ifndef ___Class_CvUtil
#define ___Class_CvUtil

#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Core>

#include "Combination.h"

class CvUtil {
public:
	static cv::Mat drawCorrespondence(const cv::Mat& image1, const cv::Mat& image2, const std::vector<CombinationPointer>& combinationPtrs, const std::vector<Eigen::MatrixXi*> positionPtrs1, const std::vector<Eigen::MatrixXi*> positionPtrs2);
	static cv::Mat resize(const cv::Mat& src, double pow);
};

#endif

