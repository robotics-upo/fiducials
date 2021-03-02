/*
 * Copyright (c) 2017-9, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <fiducial_slam/helpers.h>
#include <fiducial_slam/map.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <string>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <boost/filesystem.hpp>


static double systematic_error = 0.01;

// Constructor for map
Map::Map(ros::NodeHandle &nh): BaseMap() {
    isInitializingMap = false;

    nh.param<bool>("publish_6dof_pose", publish_6dof_pose, false);
    
    std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
    overridePublishedCovariance = nh.getParam("covariance_diagonal", covarianceDiagonal);
    
    if (overridePublishedCovariance) {
        if (covarianceDiagonal.size() != 6) {
            ROS_WARN("ignoring covariance_diagonal because it has %ld elements, not 6", covarianceDiagonal.size());
            overridePublishedCovariance = false;
        }
        // Check to make sure that the diagonal is non-zero
        for (auto variance : covarianceDiagonal) {
            if (variance == 0) {
                ROS_WARN("ignoring covariance_diagonal because it has 0 values");
                std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
                break;
            }
        }
    }

    // threshold of object error for using multi-fidicial pose
    // set -ve to never use
    nh.param<double>("multi_error_theshold", multiErrorThreshold, -1);

    nh.param<std::string>("map_file", mapFilename,
                          std::string(getenv("HOME")) + "/.ros/slam/map.txt");

    publishMarkers();
}

// Update map with a set of observations

void Map::update(std::vector<Observation> &obs, const ros::Time &time) {
    ROS_INFO("Updating map with %d observations. Map has %d fiducials", (int)obs.size(),
             (int)fiducials.size());

    frameNum++;

    if (obs.size() > 0 && fiducials.size() == 0) {
        isInitializingMap = true;
    }

    if (isInitializingMap) {
        autoInit(obs, time);
    } else if (obs.size() > 0) {
        tf2::Stamped<TransformWithVariance> T_mapCam;
        T_mapCam.frame_id_ = mapFrame;

        // if (updatePose(obs, time, T_mapCam) > 0 && obs.size() > 1 && !readOnly) {
        int ret = updatePose(obs, time, T_mapCam);
        if (obs.size() > 0 && !readOnly) {
            if (ret > 0 || use_external_loc)
                updateMap(obs, time, T_mapCam);
        }
    }

    handleAddFiducial(obs);

    publishMap();
}

// update estimates of observed fiducials from previously estimated
// camera pose

void Map::updateMap(const std::vector<Observation> &obs, const ros::Time &time,
                    const tf2::Stamped<TransformWithVariance> &T_mapCam) {
    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        f.visible = false;
    }

    for (const Observation &o : obs) {
        // This should take into account the variances from both
        tf2::Stamped<TransformWithVariance> T_mapFid = T_mapCam * o.T_camFid;
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

        if (fiducials.find(o.fid) == fiducials.end()) {
            bool add = true;
            if (use_external_loc) {
                ROS_INFO("Fid: %d:\tO Distance: %f",o.fid,  o.getDistance());
                add = o.getDistance() < 2.0;
            }
            if (add) {
                ROS_INFO("New fiducial %d", o.fid);
                fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
            } else {
                continue;
            }
            
        }
        Fiducial &f = fiducials[o.fid];
        f.visible = true;
        if (std::abs(f.pose.variance) < 1e-9) {
                f.pose.variance = 0.01;
        }
        f.update(T_mapFid);
        f.numObs++;

        for (const Observation &observation : obs) {
            int fid = observation.fid;
            if (f.id != fid) {
                f.links.insert(fid);
            }
        }
        BaseMap::publishMarker(o.fid);
    }
}

// update pose estimate of robot.  

// In the previous version: We combine the camera->base_link
// tf to each estimate so we can evaluate how good they are.  A good
// estimate would have z == roll == pitch == 0.

// Currently we use the error of the fiducial observation
int Map::updatePose(std::vector<Observation> &obs, const ros::Time &time,
                    tf2::Stamped<TransformWithVariance> &T_mapCam) {
    int numEsts = 0;
    tf2::Stamped<TransformWithVariance> T_camBase;
    tf2::Stamped<TransformWithVariance> T_baseCam;
    tf2::Stamped<TransformWithVariance> T_mapBase;

    if (use_external_loc) {
        if (lookupTransform(mapFrame, obs[0].T_camFid.frame_id_, ros::Time(0), T_mapCam.transform)) {                              
            tf2::Vector3 c = T_mapCam.transform.getOrigin();
            ROS_INFO("Obtained external transform:   %lf %lf %lf", c.x(), c.y(), c.z());
        }

        T_mapCam.variance = last_pose_variance;

    }

    if (obs.size() == 0) {
        return 0;
    }

    if (lookupTransform(obs[0].T_camFid.frame_id_, baseFrame, time, T_camBase.transform)) {
        tf2::Vector3 c = T_camBase.transform.getOrigin();
        ROS_INFO("camera->base   %lf %lf %lf", c.x(), c.y(), c.z());
        T_camBase.variance = 1.0;
    } else {
        ROS_ERROR("Cannot determine tf from camera to robot\n");
    }

    if (lookupTransform(baseFrame, obs[0].T_camFid.frame_id_, time, T_baseCam.transform)) {
        tf2::Vector3 c = T_baseCam.transform.getOrigin();
        ROS_INFO("base->camera   %lf %lf %lf", c.x(), c.y(), c.z());
        T_baseCam.variance = 1.0;
    } else {
        ROS_ERROR("Cannot determine tf from robot to camera\n");
        return numEsts;
    }

    for (Observation &o : obs) {
        if (fiducials.find(o.fid) != fiducials.end()) {
            const Fiducial &fid = fiducials[o.fid];

            tf2::Stamped<TransformWithVariance> p = fid.pose * o.T_fidCam;

            p.frame_id_ = mapFrame;
            p.stamp_ = o.T_fidCam.stamp_;

            p.setData(p * T_camBase);
            auto position = p.transform.getOrigin();
            double roll, pitch, yaw;
            p.transform.getBasis().getRPY(roll, pitch, yaw);

            // Create variance according to how well the robot is upright on the ground
            // TODO: Create variance for each DOF
            // TODO: Take into account position according to odom
            // auto cam_f = o.T_camFid.transform.getOrigin();
            // double s1 = std::pow(position.z() / cam_f.z(), 2) *
            //             (std::pow(cam_f.x(), 2) + std::pow(cam_f.y(), 2));
            // double s2 = position.length2() * std::pow(std::sin(roll), 2);
            // double s3 = position.length2() * std::pow(std::sin(pitch), 2);
            // p.variance = s1 + s2 + s3 + systematic_error;
            // o.T_camFid.variance = p.variance;

            // Variance according to the observed variance
            p.variance = o.T_fidCam.variance;

            ROS_INFO("Pose %d %lf %lf %lf %lf %lf %lf %lf", o.fid, position.x(), position.y(),
                     position.z(), roll, pitch, yaw, p.variance);

            // drawLine(fid.pose.getOrigin(), o.position);

            if (std::isnan(position.x()) || std::isnan(position.y()) || std::isnan(position.z())) {
                ROS_WARN("Skipping NAN estimate\n");
                continue;
            };

            // compute base_link pose based on this estimate

            if (numEsts == 0) {
                T_mapBase = p;
            } else {
                T_mapBase.setData(averageTransforms(T_mapBase, p));
                T_mapBase.stamp_ = p.stamp_;
            }
            numEsts++;
        }
    }

    if (numEsts == 0) {
        ROS_INFO("Finished frame - no estimates\n");

        return numEsts;
    }


    if (use_external_loc && !readOnly) {
        // Add the external localization to the mix
        tf2::Stamped<TransformWithVariance> p;
         if (lookupTransform(mapFrame, baseFrame, ros::Time(0), p.transform)) {                              
            tf2::Vector3 c = T_mapCam.transform.getOrigin();
            ROS_INFO("Obtained external transform:   %lf %lf %lf", c.x(), c.y(), c.z());
        }
        p.variance = last_pose_variance;
        T_mapBase.setData(averageTransforms(T_mapBase, p));
        T_mapBase.stamp_ = p.stamp_;
    }

    // New scope for logging vars
    {
        tf2::Vector3 trans = T_mapBase.transform.getOrigin();
        double r, p, y;
        T_mapBase.transform.getBasis().getRPY(r, p, y);

        ROS_INFO("Pose ALL %lf %lf %lf %lf %lf %lf %f", trans.x(), trans.y(), trans.z(), r, p, y,
                 T_mapBase.variance);
    }

    tf2::Stamped<TransformWithVariance> basePose = T_mapBase;
    basePose.frame_id_ = mapFrame;
    auto robotPose = toPose(basePose);

    if (overridePublishedCovariance) {
        for (int i = 0; i <= 5; i++) {
            robotPose.pose.covariance[i * 7] = covarianceDiagonal[i];  // Fill the diagonal
        }
    }

    T_mapCam = T_mapBase * T_baseCam;

    robotPosePub.publish(robotPose);

    tf2::Stamped<TransformWithVariance> outPose = basePose;
    outPose.frame_id_ = mapFrame;
    std::string outFrame = baseFrame;

    if (!odomFrame.empty()) {
        tf2::Transform odomTransform;
        if (lookupTransform(odomFrame, baseFrame, outPose.stamp_, odomTransform)) {
            outPose.setData(basePose * odomTransform.inverse());
            outFrame = odomFrame;

            tf2::Vector3 c = odomTransform.getOrigin();
            ROS_INFO("odom   %lf %lf %lf", c.x(), c.y(), c.z());
        }
        else {
            // Don't publish anything if map->odom was requested and is unavailaable
            return numEsts;
        }
    }

    // Make outgoing transform make sense - ie only consist of x, y, yaw
    // This can be disabled via the publish_6dof_pose param, mainly for debugging
    if (!publish_6dof_pose) {
        tf2::Vector3 translation = outPose.transform.getOrigin();
        translation.setZ(0);
        outPose.transform.setOrigin(translation);
        double roll, pitch, yaw;
        outPose.transform.getBasis().getRPY(roll, pitch, yaw);
        outPose.transform.getBasis().setRPY(0, 0, yaw);
    }

    poseTf = toMsg(outPose);
    poseTf.child_frame_id = outFrame;
    havePose = true;

    if (publishPoseTf && !use_external_loc) {
        publishTf();
    }

    ROS_INFO("Finished frame. Estimates %d\n", numEsts);
    return numEsts;
}


void Map::update() {
    ros::Time now = ros::Time::now();
    if (publishPoseTf && havePose && tfPublishInterval != 0.0 &&
        (now - tfPublishTime).toSec() > tfPublishInterval) {
        publishTf();
        tfPublishTime = now;
    }
    publishMarkers();
}

// Find closest fiducial to camera

static int findClosestObs(const std::vector<Observation> &obs) {
    double smallestDist = -1;
    int closestIdx = -1;

    for (size_t i = 0; i < obs.size(); i++) {
        const Observation &o = obs[i];
        double d = o.T_camFid.transform.getOrigin().length2();
        if (smallestDist < 0 || d < smallestDist) {
            smallestDist = d;
            closestIdx = i;
        }
    }

    return closestIdx;
}

// Initialize a map from the closest observed fiducial
// Figure out the closest marker, and then figure out the
// pose of that marker such that base_link is at the origin of the
// map frame
void Map::autoInit(const std::vector<Observation> &obs, const ros::Time &time) {
    ROS_INFO("Map::autoInit --> Auto init map %d", frameNum);

    tf2::Transform T_baseCam, T_map_base;

    if (obs.size() == 0)
        return;

    if (fiducials.size() == 0) {
        int idx = findClosestObs(obs);

        if (idx == -1) {
            ROS_WARN("Could not find a fiducial to initialize map from");
            return;
        }

        if (use_external_loc && obs[idx].getDistance() > 3.0) {
            ROS_INFO("Map::autoInit --> Observation too far away to initialize the map");
            return;
        }

        const Observation &o = obs[idx];
        originFid = o.fid;

        ROS_INFO("Initializing map from fiducial %d", o.fid);

        tf2::Stamped<TransformWithVariance> T = o.T_camFid;

        if (lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_, T_baseCam)) {
            if (use_external_loc) {
                if (lookupTransform(mapFrame, baseFrame, ros::Time(0), T_map_base)) {
                    T.setData(T_map_base * T_baseCam * T);
                    fiducials[o.fid] = Fiducial(o.fid, T);
                }
            } else {
                T.setData(T_baseCam * T);
                fiducials[o.fid] = Fiducial(o.fid, T);
            }
        }

        
    } else {
        for (const Observation &o : obs) {
            if (o.fid == originFid) {
                tf2::Stamped<TransformWithVariance> T = o.T_camFid;

                tf2::Vector3 trans = T.transform.getOrigin();
                ROS_INFO("Estimate of %d from base %lf %lf %lf err %lf", o.fid, trans.x(),
                         trans.y(), trans.z(), o.T_camFid.variance);

                if (lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_,
                                    T_baseCam)) {
                    if (use_external_loc) {
                        if (lookupTransform(mapFrame, baseFrame, ros::Time(0), T_map_base)) {
                            T.setData(T_map_base * T_baseCam * T);                            
                        }
                    } else {
                        T.setData(T_baseCam * T);
                    }
                }

                fiducials[originFid].update(T);
                break;
            }
        }
    }

    if (frameNum - initialFrameNum > 10 && originFid != -1) {
        isInitializingMap = false;

        fiducials[originFid].pose.variance = 0.0;
    }
}

// Attempt to add the specified fiducial to the map

void Map::handleAddFiducial(const std::vector<Observation> &obs) {

    if (fiducialToAdd == -1) {
        return;
    }

    if (fiducials.find(fiducialToAdd) != fiducials.end()) {
        ROS_INFO("Fiducial %d is already in map - ignoring add request",
                 fiducialToAdd);
        fiducialToAdd = -1;
        return;
    }

    for (const Observation &o : obs) {
        if (o.fid == fiducialToAdd) {
            ROS_INFO("Adding fiducial_id %d to map", fiducialToAdd);


            tf2::Stamped<TransformWithVariance> T = o.T_camFid;

            // Take into account position of camera on base
            tf2::Transform T_baseCam;
            if (lookupTransform(baseFrame, o.T_camFid.frame_id_,
                                o.T_camFid.stamp_, T_baseCam)) {
                T.setData(T_baseCam * T);
            }

            // Take into account position of robot in the world if known
            tf2::Transform T_mapBase;
            if (lookupTransform(mapFrame, baseFrame, ros::Time(0), T_mapBase)) {
                T.setData(T_mapBase * T);
            }
            else {
                ROS_INFO("Placing robot at the origin");
            }

            fiducials[o.fid] = Fiducial(o.fid, T);
            fiducials[originFid].pose.variance = 0.0;
            isInitializingMap = false;

            fiducialToAdd = -1;
            return;
        }
    }

    ROS_INFO("Unable to add fiducial %d to map", fiducialToAdd);
}
