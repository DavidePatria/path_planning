#ifndef PATH_planning_H
#define PATH_planning_H

#include "ros/ros.h"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

class Planner {

public:
	Planner();
	~Planner();

	void initStart();
	void setStart();
	void setGoal(double x, double y, double z);
	void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);

	void replan();
	void plan();
	
private:

	// construct the state space we are planning in
	ompl::base::StateSpacePtr space;
	// construct an instance of  space information from this state space
	ompl::base::SpaceInformationPtr si;
	// create a problem instance
	ompl::base::ProblemDefinitionPtr pdef;
	ompl::geometric::PathGeometric* path_smooth = NULL;
	// goal state
	double prev_goal[3];

	bool replan_flag = false;

	std::shared_ptr<fcl::CollisionGeometry> Quadcopter;
	std::shared_ptr<fcl::CollisionGeometry> tree_obj;
	// Flag for initialization
	bool set_start = false;
	bool isStateValid(const ompl::base::State *state);
	ompl::base::OptimizationObjectivePtr getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& space_info);

	// Callbacks
	void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, Planner* planner_ptr);
	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, Planner* planner_ptr);
	void startCallback(const geometry_msgs::PointStamped::ConstPtr &msg, Planner* planner_ptr);
	void goalCallback(const geometry_msgs::PointStamped::ConstPtr &msg, Planner* planner_ptr);

};


#endif // PATH_planning_H
