/*
 * Copyright (c) 2021, Service Robotics Lab
 * All rights reserved.
 *
 * See LICENSE file for distribution details
 */

#pragma once

#include <assert.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>

#include <list>
#include <string>

#include "Fiducial.h"
#include "g2o/types/slam2d/types_slam2d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "tf2/transform_datatypes.h"

#include "base_map.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

class G2OFiducialSlam:public BaseMap {
public:
    std::vector<geometry_msgs::Pose> _odometry_poses;
    G2OFiducialSlam();

    void updateG2O(std::vector<Observation> &obs, const ros::Time &time);

    void optimize(int n_rounds = 3, bool online = true);

    bool load(const std::string &filename);

    void publishTf() const {
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.frame_id = mapFrame;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.child_frame_id = odomFrame;
        transform_stamped.transform = tf2::toMsg(_tf_map_odom);
        br.sendTransform(transform_stamped);
        transform_stamped.header.seq++;
    }

private:
    ros::Subscriber _ft_sub, _verticesSub;
    bool _use_fiducial_area_as_weight;
    double _weighting_scale;
    ros::Publisher _path_pub;
    ros::ServiceServer _optimize_service;
    tf2::Transform _last_tf_odom_base, _tf_base_cam, _tf_map_odom;
    bool _tf_base_cam_init = false;
    const int _fiducial_id_offset = 1000000;
    std::string _g2o_file;

    // G2O STUFF
    
    // allocating the optimizer
    g2o::SparseOptimizer _optimizer;
    std::unique_ptr<SlamLinearSolver> _linear_solver; 
    Eigen::Matrix3d _odometry_information;

    int _frame_num = 0;

    void transformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg);

    bool optimizeServiceService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void updateMap(const std::vector<Observation> &obs, const ros::Time &time,
                   const tf2::Stamped<TransformWithVariance> &camera_pose);

    static void get_tf_coords(double &x, double &y, double &theta, const tf2::Transform &t);       

    inline bool checkCameraTransform(const std::string &frame_id);

    static void changeTf(tf2::Transform &t, double x, double y, double theta);
};
