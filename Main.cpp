#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

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

	cout << filePath1 << endl;
	cout << filePath2 << endl;

	return 0;
}

