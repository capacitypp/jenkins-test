#include <iostream>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
	cout << "hello, world" << endl;

	MatrixXd m1 = MatrixXd::Identity(3, 3);
	MatrixXd m2 = 2 * m1;

	cout << "[m1]" << endl;
	cout << m1 << endl;

	cout << "[m2 = 2 * m1]" << endl;
	cout << m2 << endl;

	return 0;
}

