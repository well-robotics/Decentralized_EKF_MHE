#include <Spline/Bezier_simple.hpp>
#include <iostream>

Bezier::Bezier()
{
}

Bezier::~Bezier()
{
}

void Bezier::add_way_point(const Vector3d &point, double t_end)
{
	_way_points.push_back(point);
	_way_points_time.push_back(t_end);
	if (_way_points.size() > 4)
	{
		_way_points.erase(_way_points.begin());
		_way_points_time.erase(_way_points_time.begin());
	}

	// just for debug
	way_points_VectorXd.segment(0, 9) << way_points_VectorXd.segment(3, 9);
	way_points_VectorXd.segment(9, 3) << point;

	t_interval_ = _way_points_time.back() - _way_points_time.front();
}

void Bezier::interpolate_waypoint()
{
	_distances.clear();
	_nodes.clear();
	// public function to be called to synchronize the staring and ending point of the vo interval inside of the optimization window.
	if (_way_points.size() < 4)
	{
		return;
	}

	int new_control_point_index = static_cast<int>(_way_points.size()) - 1;

	int pt = new_control_point_index - 3;

	double u_interpolate_start = (double)(t_interpolate_start_ - _way_points_time.front()) / (double)t_interval_; // steps should be t_start - t_end;

	for (double i = 0; i < interpolate_num_; i++)
	{
		double u = u_interpolate_start + u_incremental_ * i;
		Vector3d node = interpolate(u, _way_points[pt], _way_points[pt + 1], _way_points[pt + 2], _way_points[pt + 3]);
		Vector3d node_diff = node - node_pre;
		node_pre = node;

		_distances.push_back(node_diff);
		_nodes.push_back(node);

		diff_VectorXd.segment<3>(0 + i * 3) << node_diff;
		nodes_VectorXd.segment<3>(0 + i * 3) << node;
	}
}

void Bezier::set_interval(double t_interpolate_start, int interpolate_num, double dt)
{
	t_interpolate_start_ = t_interpolate_start;

	u_incremental_ = dt / (double)t_interval_;
	interpolate_num_ = interpolate_num;
	nodes_VectorXd.resize(interpolate_num * 3, 1);
	nodes_VectorXd.setZero();
	diff_VectorXd.resize(interpolate_num * 3, 1);
	diff_VectorXd.setZero();
	node_pre.setZero();
}

Vector3d Bezier::interpolate(double u, const Vector3d &P0, const Vector3d &P1, const Vector3d &P2, const Vector3d &P3)
{
	Vector3d point = Vector3d::Zero();
	point = u * u * u * ((-1) * P0 + 3 * P1 - 3 * P2 + P3);
	point += u * u * (3 * P0 - 6 * P1 + 3 * P2);
	point += u * ((-3) * P0 + 3 * P1);
	point += P0;

	return point;
}

void clear()
{
}
