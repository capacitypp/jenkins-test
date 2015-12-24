#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "MatrixUtil.h"

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char** argv)
{
	if (argc != 3) {
		cerr << argv[0] << " <file path 1> <file path 2>" << endl;
		return 1;
	}

	string filePath1(argv[1]);
	string filePath2(argv[2]);

	Mat image1 = imread(filePath1);
	Mat image2 = imread(filePath2);

	Mat gray_image1, gray_image2;
	cvtColor(image1, gray_image1, CV_RGB2GRAY);
	cvtColor(image2, gray_image2, CV_RGB2GRAY);

	MatrixXd gray1 = MatrixUtil::convertGray2MatrixXd(gray_image1);
	MatrixXd gray2 = MatrixUtil::convertGray2MatrixXd(gray_image2);

	return 0;
}

