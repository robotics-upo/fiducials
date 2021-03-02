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
#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>

#include <fiducial_msgs/FiducialMapEntry.h>
#include <fiducial_msgs/FiducialMapEntryArray.h>

#include <list>
#include <string>

#include <fiducial_slam/transform_with_variance.h>

#include "Fiducial.h"
#include "Observation.h"

#include "base_map.h"

// Class containing map data
class Map:public BaseMap {
public:
    bool publish_6dof_pose;
    double multiErrorThreshold;

    bool isInitializingMap;
    
    bool overridePublishedCovariance;
    std::vector<double> covarianceDiagonal;

    Map(ros::NodeHandle &nh);
    void update();
    void update(std::vector<Observation> &obs, const ros::Time &time);
    void autoInit(const std::vector<Observation> &obs, const ros::Time &time);
    int updatePose(std::vector<Observation> &obs, const ros::Time &time,
                   tf2::Stamped<TransformWithVariance> &cameraPose);
    void updateMap(const std::vector<Observation> &obs, const ros::Time &time,
                   const tf2::Stamped<TransformWithVariance> &cameraPose);
    void handleAddFiducial(const std::vector<Observation> &obs);
};

#endif
