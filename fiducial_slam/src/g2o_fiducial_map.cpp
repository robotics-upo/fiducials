/*
 * Copyright (c) 2021, Service Robotics Lab
 * All rights reserved.
 *
 * See LICENSE file for distribution details
 */
#include "fiducial_slam/g2o_fiducial_map.h"
#include <fiducial_slam/helpers.h>

using namespace std;
using namespace g2o;

G2OFiducialMap::G2OFiducialMap() {
    // If set, use the fiducial area in pixels^2 as an indication of the
    // 'goodness' of it. This will favor fiducials that are close to the
    // camera and center of the image. The reciprical of the area is actually
    // used, in place of reprojection error as the estimate's variance
    ros::NodeHandle nh, pnh("~");


    pnh.param<bool>("use_fiducial_area_as_weight", _use_fiducial_area_as_weight, false);
    // Scaling factor for weighing
    pnh.param<double>("weighting_scale", _weighting_scale, 1.0);
    pnh.param<double>("max_observation_distance", _max_obs_dist, 2.0);

    _ft_sub = nh.subscribe("/fiducial_transforms", 1, &G2OFiducialMap::transformCallback, this);

    _optimize_service = pnh.advertiseService("optimize", &G2OFiducialMap::optimizeService, this);

    // G2O initialization
    _linear_solver = ::make_unique<SlamLinearSolver>();
    _linear_solver->setBlockOrdering(false);
    g2o::OptimizationAlgorithmGaussNewton * solver =new g2o::OptimizationAlgorithmGaussNewton(::make_unique<SlamBlockSolver>(std::move(_linear_solver)));
    _optimizer.setAlgorithm(solver);

    if (pnh.hasParam("stats_file")) {
        pnh.getParam("stats_file", _g2o_file);
    } else {
        _g2o_file = "";
    }

    // Information of odometry, etc.
    if (load(_g2o_file))
        ROS_INFO("Stats file %s loaded successfully", _g2o_file.c_str());

    ROS_INFO("G2O Fiducial Map ready");
}

bool G2OFiducialMap::load(const std::string &filename) {
    ROS_INFO("Loading: %s", filename.c_str());
    bool ret_val = false;
    _optimizer.load(filename.c_str());
    _optimizer.optimize(3);
    fiducials.clear();
    int max_id = -1;
    for (auto &x:_optimizer.vertices()) {
        auto y = dynamic_cast<VertexSE2 *> ( x.second);
        auto &pos = y->estimate();
        if (y == nullptr) continue;
        if (x.first > 0) {   // Currently, FID 0 is not allowed!!!
            // Create
            int fid = x.first;

            tf2::Stamped<TransformWithVariance> fid_pose;
            changeTf(fid_pose.transform, pos[0], pos[1], pos[2]);
            fid_pose.frame_id_ = mapFrame;
            fid_pose.stamp_ = ros::Time::now();
            fid_pose.variance = 0.1;
            Fiducial curr_fid( fid, fid_pose);
            fiducials[fid] = curr_fid;
            
            ROS_INFO ("Adding fid %d. x = %f. y = %f. yaw = %f", fid, pos[0], pos[1], pos[2]);
        }
    }
    publishMarkers(); 
    ros::spinOnce(); 
    sleep(1);

    _frame_num = 1;
    
    return ret_val;
  }

void G2OFiducialMap::transformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg) {
    vector<Observation> observations;

    checkCameraTransform(msg->header.frame_id);

    for (size_t i = 0; i < msg->transforms.size(); i++) {
        const fiducial_msgs::FiducialTransform &ft = msg->transforms[i];

        tf2::Vector3 tvec(ft.transform.translation.x, ft.transform.translation.y,
                          ft.transform.translation.z);

        tf2::Quaternion q(ft.transform.rotation.x, ft.transform.rotation.y, ft.transform.rotation.z,
                          ft.transform.rotation.w);

        double variance;
        if (_use_fiducial_area_as_weight) {
            variance = _weighting_scale / ft.fiducial_area;
        } else {
            variance = _weighting_scale * ft.object_error;
        }

        double dist = tvec.length();
                                            
        // Only include the observation within a range:
        if (dist < _max_obs_dist) {
            Observation obs(ft.fiducial_id, tf2::Stamped<TransformWithVariance>(
                                            TransformWithVariance(ft.transform, variance),
                                            msg->header.stamp, msg->header.frame_id));
        
            observations.push_back(obs);
        }
    }
    
    updateG2O(observations, msg->header.stamp);
}

bool G2OFiducialMap::optimizeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Calling optimize");
    optimize();

    return true;
}

void G2OFiducialMap::updateG2O(std::vector<Observation> &obs, const ros::Time &time) {
    ROS_INFO("Updating map with %d observations. Map has %d fiducials", (int)obs.size(),
             (int)fiducials.size());
    // Get the current pose of the vehicle
    tf2::Stamped<TransformWithVariance> tf_map_base,tf_odom_base;
    if (lookupTransform(odomFrame, baseFrame, ros::Time(0), tf_odom_base.transform) && 
        lookupTransform(mapFrame, baseFrame, ros::Time(0), tf_map_base.transform) ) {
        if (obs.size() > 0) {
            // Add robot position (from map)
            if (_optimizer.vertices().size() == 0) {
                const SE2 robot_pose(0, 0, 0);
                VertexSE2* robot = new VertexSE2;
                robot->setId(0);
                robot->setEstimate(robot_pose);
                _optimizer.addVertex(robot);
            }
                
            updateMap(obs, time, tf_map_base);
            
        }
        _frame_num++;
    } else {
        ROS_ERROR("Transform problem!! (no map frame");
    }
    
}

void G2OFiducialMap::updateMap(const std::vector<Observation> &obs, const ros::Time &time,
                   const tf2::Stamped<TransformWithVariance> &robot_pose) 
{
    // Set not visible by default
    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        f.visible = false;
    }

    for (const Observation &o : obs) {
        // This should take into account the variances from both
        tf2::Stamped<TransformWithVariance> T_base_cam;
        T_base_cam.frame_id_ = "base_link";
        T_base_cam.transform = _tf_base_cam;
        T_base_cam.variance = 0.0004;
        tf2::Stamped<TransformWithVariance> T_mapFid = robot_pose * T_base_cam * o.T_camFid;
        T_mapFid.frame_id_ = mapFrame; 

        // New scope for logging vars
        {
            tf2::Vector3 trans = T_mapFid.transform.getOrigin();

            ROS_INFO("Estimate of %d %lf %lf %lf var %lf %lf", o.fid, trans.x(), trans.y(),
                     trans.z(), o.T_camFid.variance, T_mapFid.variance);

            if (std::isnan(trans.x()) || std::isnan(trans.y()) || std::isnan(trans.z())) {
                ROS_WARN("Skipping NAN estimate\n");
                continue;
            };
        }

        double x, y, theta;
        get_tf_coords(x, y, theta, T_mapFid.transform);
        const SE2 fid_pose(x, y, theta);
        VertexSE2* fid = new VertexSE2;

        if (fiducials.find(o.fid) == fiducials.end()) {
            ROS_INFO("New fiducial %d", o.fid);
            fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
            
            fid->setId(o.fid);
            fid->setEstimate(fid_pose);
            _optimizer.addVertex(fid);
        }
        Fiducial &f = fiducials[o.fid];
        f.visible = true;
        if (std::abs(f.pose.variance) < 1e-9) { // Handle negative variance
                f.pose.variance = 0.01;
        }

        // Constraint between robot and fiducial!
        Eigen::Matrix3d covariance; covariance.fill(0.);
        covariance(0, 0) = T_mapFid.variance;
        covariance(1, 1) = T_mapFid.variance;
        covariance(2, 2) = T_mapFid.variance;

        Eigen::Matrix3d information = covariance.inverse();

        EdgeSE2* fid_obs = new EdgeSE2;
        fid_obs->vertices()[0] = _optimizer.vertex(0);
        fid_obs->vertices()[1] = _optimizer.vertex(o.fid);

        get_tf_coords(x, y, theta, T_mapFid.transform);
        const SE2 fid_meas(x,y,theta);
        fid_obs->setMeasurement(fid_meas);
        fid_obs->setInformation(information);
        _optimizer.addEdge(fid_obs);

        // Constraints inter-fiducials (more than one fiducial at one time)
        for (const Observation &observation : obs) {
            int fid = observation.fid;
            if (f.id > fid) {
                f.links.insert(fid);
                // TODO: Add a loose constraint in the optimizer
            }
        }

        auto marker_vector = fiducials[o.fid].getMarkers(mapFrame);
        for (auto x:marker_vector) {
            markerPub.publish(x);
        }
    }
}

void G2OFiducialMap::get_tf_coords(double &x, double &y, double &theta, const tf2::Transform &t) {
    x = t.getOrigin().getX();
    y = t.getOrigin().getY();
    auto q = t.getRotation();
    tf2::Matrix3x3 M(q);
    double roll,pitch;
    M.getRPY(roll, pitch, theta);
}

bool G2OFiducialMap::checkCameraTransform(const std::string &frame_id) {
    if (!_tf_base_cam_init)
        this->lookupTransform(baseFrame, frame_id, ros::Time(0), _tf_base_cam);

    return _tf_base_cam_init;

}

void G2OFiducialMap::changeTf(tf2::Transform &t, double x, double y, double theta) {
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    auto q = t.getRotation();
    tf2::Matrix3x3 M(q);
    double roll, pitch, yaw;
    M.getRPY(roll, pitch, yaw);
    q.setRPY(roll, pitch, theta);
    t.setRotation(q);
}


/*********************************************************************************
   * optimization
   ********************************************************************************/
void G2OFiducialMap::optimize(int n_rounds, bool online) {
  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(_optimizer.vertex(0));
  if (firstRobotPose == nullptr) {
      ROS_ERROR("Could not get the first Robot Pose. Cancelling\n");
      return;
  }
  firstRobotPose->setFixed(true);
  _optimizer.setVerbose(true);

  ROS_INFO("Optimizing");
  _optimizer.initializeOptimization();
  _optimizer.optimize(n_rounds);
  ROS_INFO("Done Optimizing.");

  for (auto &x:_optimizer.vertices()) {
    if (x.first == 0)
        continue;
    auto y = dynamic_cast<VertexSE2 *> ( x.second);
    auto &pos = y->estimate();
    if (y == nullptr) continue;
    
    // Updating fid
    int fid = x.first;
    
    Fiducial &curr_fid = fiducials[fid];
    changeTf(curr_fid.pose.transform, pos[0], pos[1], pos[2]);
    // ROS_INFO ("Updating fid %d. x = %f. y = %f. yaw = %f", fid, pos[0], pos[1], pos[2]);
  }
  publishMarkers();

  if (_g2o_file.size() > 0) {
    _optimizer.save(_g2o_file.c_str());
  }
  
  // Save the map in the fiducial format --> so that we can use EKF to estimate the pose online for ground truth
  saveMap();
}

void G2OFiducialMap::estimatedPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    Eigen::Matrix3d covariance;
    covariance.fill(0.);
    covariance(0, 0) = msg->pose.covariance[0];
    covariance(1, 1) = msg->pose.covariance[7]; 
    covariance(2, 2) = msg->pose.covariance[35];

    _map_information = covariance.inverse();
    _map_pose_variance = msg->pose.covariance[0] + msg->pose.covariance[7];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "g2o_fiducial_map");
    
    sleep(2); // Let the systems initialize
    std::unique_ptr<G2OFiducialMap> node;
    node.reset(new G2OFiducialMap);
    double rate = 20.0;
    ros::Rate r(rate);
    
    int update_secs = 100;

    ros::NodeHandle pnh("~");
    
    int n_iter;
    pnh.param("n_iter", n_iter, 3);
    pnh.param("update_secs", update_secs, 10);
    const int update_iters = update_secs*rate;
    int cont = 0;

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        // node->publishMarkers();
        cont ++;

        if (cont%100 == 0)
            node->publishMarkers();

        // Once the data has been collected --> optimize with g2o
        if (node != nullptr && cont % update_iters == 0) {
            node->optimize(n_iter);
        }
    }
   
    return 0;
}

