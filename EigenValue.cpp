#include "EigenValue.h"

using namespace std;
using namespace Eigen;

vector<EigenValue> EigenValue::solve(const MatrixXd& a) throw(EigenValueException)
{
	EigenSolver<MatrixXd> es(a);
	if (es.info() != Success)
		throw EigenValueException();
	
	const EigenSolver<MatrixXd>::EigenvalueType& values = es.eigenvalues();
	const EigenSolver<MatrixXd>::EigenvectorsType& vectors = es.eigenvectors();
	vector<EigenValue> eigenvalues;
	for (int i = 0; i < values.size(); i++) {
		double value = values[i].real();
		const Matrix<EigenSolver<MatrixXd>::ComplexScalar, Dynamic, 1>& vComplex = vectors.col(i);
		MatrixXd v(a.cols(), 1);
		for (int j = 0; j < v.rows(); j++)
			v(j, 0) = vComplex(j, 0).real();
		eigenvalues.push_back(EigenValue(value, v));
	}

	sort(eigenvalues.begin(), eigenvalues.end());

	return eigenvalues;
}

vector<EigenValue> EigenValue::solve(const MatrixXd& a, const MatrixXd& g) throw(EigenValueException)
{
	vector<EigenValue> eigenvalues;
	try {
		eigenvalues = solve(g);
	}
	catch (const EigenValueException& e) {
		throw e;
	}
	for (unsigned i = 0; i < eigenvalues.size(); i++) {
		EigenValue& eigenvalue = eigenvalues[i];
		MatrixXd& eigenvector = eigenvalue.getEigenvector();
		eigenvector /= eigenvector.norm();
	}
	MatrixXd t = MatrixXd::Zero(a.cols(), a.cols());
	for (unsigned i = 0; i < eigenvalues.size(); i++) {
		const EigenValue& ev = eigenvalues[i];
		double eigenvalue = ev.getEigenvalue();
		const MatrixXd& eigenvector = ev.getEigenvector();
		t += eigenvector * eigenvector.transpose() / sqrt(eigenvalue);
	}
	MatrixXd aTilde = t * a * t;
	try {
		eigenvalues = solve(aTilde);
	}
	catch (const EigenValueException& e) {
		throw e;
	}
	for (unsigned i = 0; i < eigenvalues.size(); i++) {
		EigenValue& ev = eigenvalues[i];
		MatrixXd& eigenvector = ev.getEigenvector();
		eigenvector = t * eigenvector;
	}
	
	sort(eigenvalues.begin(), eigenvalues.end());

	return eigenvalues;
}

