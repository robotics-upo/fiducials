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
#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <fiducial_msgs/FiducialMapEntry.h>
#include <fiducial_msgs/FiducialMapEntryArray.h>

#include <list>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>
#include <fiducial_slam/AddFiducial.h>

#include <fiducial_slam/transform_with_variance.h>

#include "Fiducial.h"
#include "Observation.h"

// Class containing map data
class BaseMap {
public:
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> listener;

    ros::Publisher markerPub;
    ros::Publisher mapPub;
    ros::Publisher robotPosePub;
    ros::Publisher cameraPosePub;
    
    std::string mapFilename;
    std::string mapFrame;
    std::string odomFrame;
    std::string cameraFrame;
    std::string baseFrame;
    double future_date_transforms;

    ros::ServiceServer clearSrv;
    ros::ServiceServer addSrv;
    virtual bool clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    virtual bool addFiducialCallback(fiducial_slam::AddFiducial::Request &req,
                             fiducial_slam::AddFiducial::Response &res);

    bool use_external_loc; // If true, this is used to refine an external localization module. PublishTf should be false
    ros::Subscriber robotPoseSub; // used with External Localization --> gets the covariance estimate of the pose
    double last_pose_Dvariance; // Last covariance of the external pose estimate
    virtual void poseCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    bool readOnly;
    int frameNum;
    int initialFrameNum;
    int originFid;

    bool havePose;
    float tfPublishInterval;
    bool publishPoseTf;
    ros::Time tfPublishTime;
    geometry_msgs::TransformStamped poseTf;
    double last_pose_variance;

    std::map<int, Fiducial> fiducials;
    int fiducialToAdd;

    BaseMap();

    bool override_variance;
    double add_variance;
    virtual bool loadMap();
    virtual bool loadMap(std::string filename);
    virtual bool saveMap();
   virtual bool saveMap(std::string filename);

    virtual void publishTf();
    virtual void publishMap();
    virtual void publishMarker(int fid) const;
    virtual void publishMarkers();
    void drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1);

    virtual bool lookupTransform(const std::string &from, const std::string &to, const ros::Time &time,
                         tf2::Transform &T) const;
};

