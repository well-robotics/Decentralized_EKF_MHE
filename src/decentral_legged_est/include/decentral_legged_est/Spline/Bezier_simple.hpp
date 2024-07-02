// Edited based on (https://github.com/chen0040/cpp-spline)

#ifndef _H__Bezier_H
#define _H__Bezier_H

#include <cassert>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
class Bezier
{
public:
    Bezier();
    ~Bezier();

public:
    void add_way_point(const Vector3d &point, double t_end);
    void interpolate_waypoint();
    void clear();

    std::vector<Vector3d> _way_points;
    std::vector<double> _way_points_time;
    std::vector<Vector3d> _distances;
    std::vector<Vector3d> _nodes;
    VectorXd way_points_VectorXd = VectorXd::Zero(12);
    VectorXd nodes_VectorXd = VectorXd::Zero(3);
    VectorXd diff_VectorXd = VectorXd::Zero(3);

    double t_interval_;
    double t_interpolate_start_;
    double u_incremental_;
    double interpolate_num_;

    void set_interval(double t_interpolate_start, int interpolate_num, double dt);

    Vector3d node_pre = Vector3d::Zero();
    int _steps;
    int _steps_start;

    Vector3d node(int i) const { return _nodes[i]; }
    int node_count() const { return static_cast<int>(_nodes.size()); }

protected:
    Vector3d interpolate(double u, const Vector3d &P0, const Vector3d &P1, const Vector3d &P2, const Vector3d &P3);
};

#endif