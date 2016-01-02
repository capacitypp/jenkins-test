#ifndef ___Class_RobustImageMatching
#define ___Class_RobustImageMatching

#include <vector>

#include <Eigen/Core>

class RobustImageMatching {
	static bool isDuplicatePosition(const Eigen::MatrixXi& position1, const Eigen::MatrixXi& position2);
	static bool isDuplicatePosition(const std::vector<Eigen::MatrixXi*>& positionPtrs, const Eigen::MatrixXi& position);
	static bool isProtrudingPosition(const Eigen::MatrixXi& position, int w, int width, int height);
	static Eigen::MatrixXd computeT(const Eigen::MatrixXd& gray, const Eigen::MatrixXi& position, int w);
public:
	static std::vector<Eigen::MatrixXi*> removeDuplicatePositions(const std::vector<Eigen::MatrixXi*>& positionPtrs);
	static std::vector<Eigen::MatrixXi*> removeProtrudingPositions(const std::vector<Eigen::MatrixXi*>& positionPtrs, int w, int width, int height);
	static std::vector<Eigen::MatrixXd> computeTs(const Eigen::MatrixXd& gray, const std::vector<Eigen::MatrixXi*>& positionPtrs, int w);
};

#endif

