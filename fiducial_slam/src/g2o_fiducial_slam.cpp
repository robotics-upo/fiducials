/*
 * Copyright (c) 2021, Service Robotics Lab
 * All rights reserved.
 *
 * See LICENSE file for distribution details
 */
#include "fiducial_slam/g2o_fiducial_slam.h"

#include <nav_msgs/Path.h>

using namespace std;
using namespace g2o::tutorial;

G2OFiducialSlam::G2OFiducialSlam() {
    // If set, use the fiducial area in pixels^2 as an indication of the
    // 'goodness' of it. This will favor fiducials that are close to the
    // camera and center of the image. The reciprical of the area is actually
    // used, in place of reprojection error as the estimate's variance
    ros::NodeHandle nh, pnh("~");


    pnh.param<bool>("use_fiducial_area_as_weight", _use_fiducial_area_as_weight, false);
    // Scaling factor for weighing
    pnh.param<double>("weighting_scale", _weighting_scale, 1.0);

    _ft_sub = nh.subscribe("/fiducial_transforms", 1, &G2OFiducialSlam::transformCallback, this);

    _path_pub = ros::Publisher(pnh.advertise<nav_msgs::Path>("/path", 100));

    _optimize_service = pnh.advertiseService("optimize", &G2OFiducialSlam::optimizeServiceService, this);

    // G2O initialization
    _linear_solver = g2o::make_unique<SlamLinearSolver>();
    _linear_solver->setBlockOrdering(false);
    g2o::OptimizationAlgorithmGaussNewton * solver =new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(_linear_solver)));
    _optimizer.setAlgorithm(solver);

    if (pnh.hasParam("stats_file")) {
        pnh.getParam("stats_file", _g2o_file);
    } else {
        _g2o_file = "";
    }

    // Information of odometry, etc.
    double rot_noise, trans_noise_x, trans_noise_y;
    pnh.param<double>("rot_noise", rot_noise, 0.05);
    pnh.param<double>("trans_noise_x", trans_noise_x, 0.05);
    pnh.param<double>("trans_noise_y", trans_noise_y, 0.02);
    Eigen::Matrix3d covariance;
    covariance.fill(0.);
    covariance(0, 0) = trans_noise_x*trans_noise_x;
    covariance(1, 1) = trans_noise_y*trans_noise_y; 
    covariance(2, 2) = rot_noise*rot_noise;
    _odometry_information = covariance.inverse();

    _tf_map_odom.setOrigin(tf2::Vector3(0,0,0));
    _tf_map_odom.setRotation(tf2::Quaternion(0,0,0,1));

    if (load(_g2o_file))
        ROS_INFO("Stats file %s loaded successfully", _g2o_file.c_str());

    ROS_INFO("G2O Fiducial Slam ready");
}

bool G2OFiducialSlam::load(const std::string &filename) {
    // Initialize path for visualization 
    nav_msgs::Path p;
    static int path_seq = 0;
    p.header.frame_id = mapFrame;
    p.header.stamp = ros::Time::now();
    p.header.seq = path_seq++;

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
        if (x.first >= _fiducial_id_offset) {
            // Create
            int fid = x.first - _fiducial_id_offset;

            tf2::Stamped<TransformWithVariance> fid_pose;
            changeTf(fid_pose.transform, pos[0], pos[1], pos[2]);
            fid_pose.frame_id_ = mapFrame;
            fid_pose.stamp_ = ros::Time::now();
            fid_pose.variance = 0.1;
            Fiducial curr_fid( fid, fid_pose);
            fiducials[fid] = curr_fid;
            
            ROS_INFO ("Adding fid %d. x = %f. y = %f. yaw = %f", fid, pos[0], pos[1], pos[2]);
        } else {
            tf2::Transform t;
            changeTf(t, pos[0], pos[1], pos[2]);
            geometry_msgs::PoseStamped curr_pose;
            curr_pose.header = p.header;
            curr_pose.pose.orientation = tf2::toMsg(t.getRotation());
            curr_pose.pose.position.x = pos[0];
            curr_pose.pose.position.y = pos[1];
            curr_pose.pose.position.z = 0.0;
            p.poses.push_back(curr_pose);
            max_id = std::max(max_id, x.first);
        }
    }
    publishTf();sleep(1);ros::spinOnce();
    publishMarkers(); ros::spinOnce(); sleep(1);
    _frame_num = max_id + 1;
    
    return ret_val;
  }

void G2OFiducialSlam::transformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg) {
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

        Observation obs(ft.fiducial_id, tf2::Stamped<TransformWithVariance>(
                                            TransformWithVariance(ft.transform, variance),
                                            msg->header.stamp, msg->header.frame_id));
        observations.push_back(obs);
    }
    
    updateG2O(observations, msg->header.stamp);
}

bool G2OFiducialSlam::optimizeServiceService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Calling optimize");
    optimize();

    return true;
}

void G2OFiducialSlam::updateG2O(std::vector<Observation> &obs, const ros::Time &time) {
    ROS_INFO("Updating map with %d observations. Map has %d fiducials", (int)obs.size(),
             (int)fiducials.size());
    // Get the current pose of the vehicle
    tf2::Stamped<TransformWithVariance> tf_map_base,tf_odom_base;
    if (lookupTransform(odomFrame, baseFrame, ros::Time(0), tf_odom_base.transform) && 
        lookupTransform(mapFrame, baseFrame, ros::Time(0), tf_map_base.transform) ) {
        if (obs.size() > 0) {
            tf2::Stamped<TransformWithVariance> T_odom_cam;

            // Add robot position (from map)
            double x, y, theta;
            get_tf_coords(x,y,theta, tf_map_base.transform);

            const SE2 robot_pose(x, y, theta);
            VertexSE2* robot = new VertexSE2;
            robot->setId(_frame_num);
            robot->setEstimate(robot_pose);
            _optimizer.addVertex(robot);

            if (_frame_num > 0) {
                // Not first frame --> add odometry constraint
                EdgeSE2* odometry = new EdgeSE2;
                odometry->vertices()[0] = _optimizer.vertex(_frame_num - 1);
                odometry->vertices()[1] = _optimizer.vertex(_frame_num);

                auto odom_tf = _last_tf_odom_base.inverse() * tf_odom_base.transform;
                get_tf_coords(x, y, theta, odom_tf);
                const SE2 odom_meas(x, y, theta);
                odometry->setMeasurement(odom_meas);
                odometry->setInformation(_odometry_information); 
                _optimizer.addEdge(odometry);

                 // For the constraints of the fiducials, we use updateMap function
                updateMap(obs, time, tf_map_base);
            }

           
        }
        _frame_num++;
        _last_tf_odom_base = tf_odom_base.transform;
    } else {
        ROS_ERROR("Transform problem!! (no map frame");
    }
    
}

void G2OFiducialSlam::updateMap(const std::vector<Observation> &obs, const ros::Time &time,
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
        T_base_cam.variance = 1;
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
            
            fid->setId(o.fid + _fiducial_id_offset);
            fid->setEstimate(fid_pose);
            _optimizer.addVertex(fid);
        }
        Fiducial &f = fiducials[o.fid];
        f.visible = true;
        if (std::abs(f.pose.variance) < 1e-9) { // Handle negative variance
                f.pose.variance = 0.01;
        }

        // Constrait between robot and fiducial!
        Eigen::Matrix3d covariance; covariance.fill(0.);
        covariance(0, 0) = o.T_camFid.variance*2.0;
        covariance(1, 1) = o.T_camFid.variance*2.0;
        covariance(2, 2) = o.T_camFid.variance *5.0;

        Eigen::Matrix3d information = covariance.inverse();

        EdgeSE2* fid_obs = new EdgeSE2;
        fid_obs->vertices()[0] = _optimizer.vertex(_frame_num);
        fid_obs->vertices()[1] = _optimizer.vertex(o.fid + _fiducial_id_offset);

        auto fid_tf =  _tf_base_cam * o.T_camFid;
        get_tf_coords(x, y, theta, fid_tf.transform);
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



void G2OFiducialSlam::get_tf_coords(double &x, double &y, double &theta, const tf2::Transform &t) {
    x = t.getOrigin().getX();
    y = t.getOrigin().getY();
    auto q = t.getRotation();
    tf2::Matrix3x3 M(q);
    double roll,pitch;
    M.getRPY(roll, pitch, theta);
}

bool G2OFiducialSlam::checkCameraTransform(const std::string &frame_id) {
    if (!_tf_base_cam_init)
        this->lookupTransform(baseFrame, frame_id, ros::Time(0), _tf_base_cam);

    return _tf_base_cam_init;

}

void G2OFiducialSlam::changeTf(tf2::Transform &t, double x, double y, double theta) {
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
void G2OFiducialSlam::optimize(int n_rounds, bool online) {
  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(_optimizer.vertex(0));
  if (firstRobotPose == nullptr) {
      ROS_ERROR("Could not get the first Robot Pose. Cancelling\n");
      return;
  }
  firstRobotPose->setFixed(true);
  _optimizer.setVerbose(true);

  tf2::Transform t_end, t_odom_base; // Before optimization --> get odom base

  bool got_transform = lookupTransform(odomFrame, baseFrame, ros::Time(0), t_odom_base);

  ROS_INFO("Optimizing");
  _optimizer.initializeOptimization();
  ROS_INFO("HEY");
  _optimizer.optimize(n_rounds);
  ROS_INFO("Done Optimizing.");
  int max_id = 0;

  // Initialize path for visualization 
  nav_msgs::Path p;
  static int path_seq = 0;
  p.header.frame_id = mapFrame;
  p.header.stamp = ros::Time::now();
  p.header.seq = path_seq++;

  for (auto &x:_optimizer.vertices()) {
      auto y = dynamic_cast<VertexSE2 *> ( x.second);
      auto &pos = y->estimate();
      if (y == nullptr) continue;
      if (x.first >= _fiducial_id_offset) {
          // Updating fid
          int fid = x.first - _fiducial_id_offset;
          
          Fiducial &curr_fid = fiducials[fid];
          changeTf(curr_fid.pose.transform, pos[0], pos[1], pos[2]);
          // ROS_INFO ("Updating fid %d. x = %f. y = %f. yaw = %f", fid, pos[0], pos[1], pos[2]);
      } else {
          max_id = std::max(max_id, x.first); // Get max ID
        
          tf2::Transform t;
          changeTf(t, pos[0], pos[1], pos[2]);
          geometry_msgs::PoseStamped curr_pose;
          curr_pose.header = p.header;
          curr_pose.pose.orientation = tf2::toMsg(t.getRotation());
          curr_pose.pose.position.x = pos[0];
          curr_pose.pose.position.y = pos[1];
          curr_pose.pose.position.z = 0.0;
          p.poses.push_back(curr_pose);
      }
  }
  auto y = dynamic_cast<VertexSE2 *>(_optimizer.vertices()[max_id]);
  auto pos = y->estimate();
  if (got_transform) {
    ROS_INFO("Updating map frame. max_id = %d, x = %f. y = %f. yaw = %f", max_id, pos[0], pos[1], pos[2]);
    changeTf(t_end, pos[0], pos[1], pos[2]);

    _tf_map_odom = t_end * t_odom_base.inverse();
  }

  publishMarkers();
  _path_pub.publish(p);

  if (_g2o_file.size() > 0) {
    _optimizer.save(_g2o_file.c_str());
  }
}

std::unique_ptr<G2OFiducialSlam> node;
int n_iter;

void mySigintHandler(int sig) {

    if (node != nullptr) {
        ROS_INFO("Optimizing on exit.");
        node->optimize(n_iter*10, false);
    }   


    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "g2o_fiducial_slam");
    
    sleep(2); // Let the systems initialize

    node.reset(new G2OFiducialSlam);
    double rate = 20.0;
    ros::Rate r(rate);
    
    int update_secs = 100;

    ros::NodeHandle pnh("~");
    bool emit_tf = true;
    pnh.param("emit_tf", emit_tf, true);
    pnh.param("n_iter", n_iter, 3);
    pnh.param("update_secs", update_secs, 10);
    const int update_iters = update_secs*rate;
    int cont = 0;

    signal(SIGINT, mySigintHandler);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        // node->publishMarkers();
        cont ++;

        if (emit_tf) {
            node->publishTf();
        }

        // Once the data has been collected --> optimize with g2o
        if (node != nullptr && cont % update_iters == 0) {
            node->optimize(n_iter);
        }
    }
    if (node != nullptr) {
        ROS_INFO("Optimizing on exit.");
        node->optimize(n_iter*10, false);
    }    
    sleep(10);
    return 0;
}


