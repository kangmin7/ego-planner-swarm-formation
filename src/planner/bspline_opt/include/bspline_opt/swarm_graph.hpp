#ifndef _SWARM_GRAPH_H_
#define _SWARM_GRAPH_H_

#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

// Formation-similarity metric from ZJU-FAST-Lab/Swarm-Formation (ICRA 2022,
// "Distributed Swarm Trajectory Optimization for Formation Flight in Dense
// Environments"): the swarm is a complete graph weighted by squared pairwise
// distances; the cost is the squared Frobenius norm of the difference between
// the current and desired symmetric normalized Laplacians. Invariant to
// translation, rotation and uniform scaling of the swarm.
class SwarmGraph
{

private:
    std::vector<Eigen::Vector3d> nodes;          // Current vertex positions
    std::vector<Eigen::Vector3d> nodes_des;      // Desired graph (permutation may change after assignment)
    std::vector<Eigen::Vector3d> nodes_des_init; // Initial desired graph (never changes once set)

    std::vector<Eigen::Vector3d> agent_grad; // Gradient of the cost w.r.t. each node position

    bool have_desired;

    Eigen::MatrixXd A;    // Adjacency matrix
    Eigen::VectorXd D;    // Degree matrix
    Eigen::MatrixXd Lhat; // Symmetric Normalized Laplacian

    Eigen::MatrixXd A_des;    // Desired adjacency matrix
    Eigen::VectorXd D_des;    // Desired degree matrix
    Eigen::MatrixXd Lhat_des; // Desired SNL

    Eigen::MatrixXd DLhat; // Difference of SNL

public:
    SwarmGraph();
    ~SwarmGraph() {}

    // Update the nodes, feature matrices & gradients
    bool updateGraph(const std::vector<Eigen::Vector3d> &swarm);

    // Set desired swarm nodes
    bool setDesiredForm(const std::vector<Eigen::Vector3d> &swarm_des);

    // Calculate the graph feature matrices
    bool calcMatrices(const std::vector<Eigen::Vector3d> &swarm,
                      Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                      Eigen::MatrixXd &SNL);

    // Squared euclidean distance
    double calcDist2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

    // Squared F-norm of the SNL difference
    bool calcFNorm2(double &cost);

    // Gradient of the cost over one node position
    bool calcFGrad(Eigen::Vector3d &gradp, int idx);

    // Get the id-th gradient over position
    Eigen::Vector3d getGrad(int id);
    bool getGrad(std::vector<Eigen::Vector3d> &swarm_grad);

    std::vector<Eigen::Vector3d> getDesNodesInit() { return nodes_des_init; }
    std::vector<Eigen::Vector3d> getDesNodesCur() { return nodes_des; }

    typedef std::unique_ptr<SwarmGraph> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif
