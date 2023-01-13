#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/TriangulationFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

class GPSFactor : public NoiseModelFactor1<Point3>
{
	double mx_, my_, mz_;

public:
	GPSFactor(Key j, Point3 point, const SharedNoiseModel &model)
		: NoiseModelFactor1<Point3>(model, j), mx_(point.x()), my_(point.y()), mz_(point.z()) {}

	Vector evaluateError(const Point3 &q,
						 boost::optional<Matrix &> H = boost::none) const
	{
		if (H)
			(*H) = (gtsam::Matrix(3, 3) << 1, 0.0, 0.0,
					0.0, 1, 0.0,
					0.0, 0.0, 1)
					   .finished();
		return (Vector(3) << q.x() - mx_, q.y() - my_, q.z() - mz_).finished();
	}
};

class GraphOptimizationNode
{
public:
	ros::NodeHandle nh;

	ros::Subscriber velocities_sub;
	ros::Subscriber pose_sub;

	ros::Publisher optimized_path_pub;

	ros::Timer optimization_timer;

	geometry_msgs::TwistStamped::ConstPtr velocities_msg;
	geometry_msgs::PoseWithCovarianceStampedConstPtr pose_msg;

	ros::Time last_graph_added;
	NonlinearFactorGraph graph;
	Values initialEstimate;
	std::string uav_name;
	int key_index = 1;

	GraphOptimizationNode()
	{
		ros::NodeHandle param_nh("~");
		param_nh.getParam("uav_name", uav_name);
		std::cout << uav_name << std::endl;
		velocities_sub = nh.subscribe("/" + uav_name + "/mavros/local_position/velocity_local",
									  1, &GraphOptimizationNode::velocities_callback, this);
		pose_sub = nh.subscribe("/" + uav_name + "/noisy/pose_cov",
								1, &GraphOptimizationNode::pose_callback, this);

		optimized_path_pub = nh.advertise<nav_msgs::Path>("/" + uav_name + "/optimized/path", 10);

		optimization_timer = nh.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent &event)
											{ this->optimize(); });

		init_graph();
	}

	void init_graph()
	{
		Point3 prior(0.0, 0.0, 0.0);
		noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
		graph.emplace_shared<PriorFactor<Point3>>(0, prior, priorNoise);
		initialEstimate.insert(0, prior);

		last_graph_added = ros::Time::now();
	}

	void publish_path(Values results)
	{
		auto msg = nav_msgs::Path();
		msg.header.frame_id = "map";
		int first_key = results.keys().at(0);

		for (int i = first_key; i < results.size(); i++)
		{
			geometry_msgs::PoseStamped pose;
			auto point = results.at(i).cast<Point3>();
			pose.pose.position.x = point.x();
			pose.pose.position.y = point.y();
			pose.pose.position.z = point.z();

			msg.poses.push_back(pose);
		}

		optimized_path_pub.publish(msg);
	}

	void velocities_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		this->velocities_msg = msg;
	}

	void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
	{
		this->pose_msg = msg;
	}

	void optimize()
	{
		if (!velocities_msg || !pose_msg)
			return;

		auto timestamp = get_timestamp();
		auto odometry = calc_shift(timestamp);
		auto pose_point = get_pose_point();	

		noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.2));
		graph.emplace_shared<BetweenFactor<Point3>>(key_index - 1, key_index, odometry, odometryNoise);

		noiseModel::Diagonal::shared_ptr gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 1));
		graph.emplace_shared<GPSFactor>(key_index, pose_point, gpsNoise);

		initialEstimate.insert(key_index, pose_point);
		key_index++;

		LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
		Values result = optimizer.optimize();
		publish_path(result);
	}

	double get_timestamp()
	{
		auto now = ros::Time::now();
		double timestamp = (now - last_graph_added).toSec();
		last_graph_added = now;

		return timestamp;
	}

	Point3 calc_shift(double timestamp)
	{
		auto velocities = velocities_msg->twist.linear;
		float x = velocities.x * timestamp;
		float y = velocities.y * timestamp;
		float z = velocities.z * timestamp;

		return Point3(x, y, z);
	}

	Point3 get_pose_point()
	{
		auto point = pose_msg->pose.pose.position;

		return Point3(point.x, point.y, point.z);
	}
};

int main(int argc, char **argv)
{
	ros::Time::init();
	ros::init(argc, argv, "graph_optimization_node");
	auto rate = ros::Rate(30);
	ROS_INFO("Start graph_optimization_node");

	GraphOptimizationNode node;

	while (ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}