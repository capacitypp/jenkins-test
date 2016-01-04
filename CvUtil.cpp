#include <opencv2/imgproc.hpp>

#include "CvUtil.h"

using namespace std;
using namespace cv;
using namespace Eigen;

Mat CvUtil::drawCorrespondence(const Mat& image1, const Mat& image2, const vector<CombinationPointer>& combinationPtrs, const vector<MatrixXi*> positionPtrs1, const vector<MatrixXi*> positionPtrs2)
{
	Mat image;
	cv::addWeighted(image1, 0.5, image2, 0.5, 0.0, image);
	for (unsigned i = 0; i < combinationPtrs.size(); i++) {
		const CombinationPointer& combinationPtr = combinationPtrs[i];
		Combination* ptr = combinationPtr.getPointer();
		const MatrixXi& position1 = *positionPtrs1[ptr->getP()];
		const MatrixXi& position2 = *positionPtrs2[ptr->getQ()];
		Point point1(position1(0, 0), position1(1, 0));
		Point point2(position2(0, 0), position2(1, 0));
		Scalar color(255, 255, 255);
		line(image, point1, point2, color, 5);
	}
	return image;
}

Mat CvUtil::resize(const Mat& src, double pow)
{
	Size size(src.cols * pow, src.rows * pow);
	Mat image;
	cv::resize(src, image, size);
	return image;
}

