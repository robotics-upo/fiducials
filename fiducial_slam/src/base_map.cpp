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

BaseMap::BaseMap() : tfBuffer(ros::Duration(30.0)) {
    ros::NodeHandle nh("~");
    frameNum = 0;
    initialFrameNum = 0;
    originFid = -1;
    havePose = false;
    fiducialToAdd = -1;

    listener = make_unique<tf2_ros::TransformListener>(tfBuffer);

    robotPosePub =
        ros::Publisher(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_pose", 1));
    cameraPosePub = ros::Publisher(
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_slam/camera_pose", 1));

    markerPub = ros::Publisher(nh.advertise<visualization_msgs::Marker>("/fiducials", 100));
    mapPub = ros::Publisher(nh.advertise<fiducial_msgs::FiducialMapEntryArray>("/fiducial_map", 1));

    clearSrv = nh.advertiseService("clear_map", &Map::clearCallback, this);
    addSrv = nh.advertiseService("add_fiducial", &Map::addFiducialCallback, this);

    robotPoseSub = nh.subscribe("in_pose",1, &Map::poseCovarianceCallback, this);

    nh.param<std::string>("map_frame", mapFrame, "map");
    nh.param<std::string>("odom_frame", odomFrame, "odom");
    nh.param<std::string>("base_frame", baseFrame, "base_footprint");

    nh.param<float>("tf_publish_interval", tfPublishInterval, 1.0);
    nh.param<bool>("publish_tf", publishPoseTf, true);
    nh.param<double>("systematic_error", systematic_error, 0.01);
    nh.param<double>("future_date_transforms", future_date_transforms, 0.1);
    nh.param<bool>("read_only_map", readOnly, false);

    // External loc additionss
    nh.param<bool>("use_external_loc", use_external_loc, false);

    // Parameters for loading maps
    nh.param<double>("add_variance", add_variance, 0.0);
    nh.param<bool>("override_variance", override_variance, false);
    
    nh.param<std::string>("map_file", mapFilename,
                          std::string(getenv("HOME")) + "/.ros/slam/map.txt");

    boost::filesystem::path mapPath(mapFilename);
    boost::filesystem::path dir = mapPath.parent_path();
    boost::filesystem::create_directories(dir);

    std::string initialMap;
    nh.param<std::string>("initial_map_file", initialMap, "");

    if (!initialMap.empty()) {
        loadMap(initialMap);
    } else {
        loadMap();
    }

    publishMarkers();
}

// lookup specified transform
bool BaseMap::lookupTransform(const std::string &from, const std::string &to, const ros::Time &time,
                          tf2::Transform &T) const {
    geometry_msgs::TransformStamped transform;

    try {
        // transform = tfBuffer.lookupTransform(from, to, time);
        transform = tfBuffer.lookupTransform(from, to, ros::Time(0));

        tf2::fromMsg(transform.transform, T);
        tf2::Vector3 c = T.getOrigin();
        

        return !isnan(c.x()) && !isnan(c.y()) && !isnan(c.z());
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
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

// Publish map -> odom tf

void BaseMap::publishTf() {
    tfPublishTime = ros::Time::now();
    poseTf.header.stamp = tfPublishTime + ros::Duration(future_date_transforms);
    broadcaster.sendTransform(poseTf);
}

// save map to file

bool BaseMap::saveMap() { return saveMap(mapFilename); }

bool BaseMap::saveMap(std::string filename) {
    ROS_INFO("Saving map with %d fiducials to file %s\n", (int)fiducials.size(), filename.c_str());

    FILE *fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for write\n", filename.c_str());
        return false;
    }

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        tf2::Vector3 trans = f.pose.transform.getOrigin();
        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);

        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %d", f.id, trans.x(), trans.y(), trans.z(),
                rad2deg(rx), rad2deg(ry), rad2deg(rz), f.pose.variance, f.numObs);

        for (const auto linked_fid : f.links) {
            fprintf(fp, " %d", linked_fid);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}


// Load map from file
bool BaseMap::loadMap() { return loadMap(mapFilename); }

bool BaseMap::loadMap(std::string filename) {
    int numRead = 0;

    ROS_INFO("Load map %s", filename.c_str());

    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for read\n", filename.c_str());
        return false;
    }

    const int BUFSIZE = 2048;
    char linebuf[BUFSIZE];
    char linkbuf[BUFSIZE];

    while (!feof(fp)) {
        if (fgets(linebuf, BUFSIZE - 1, fp) == NULL) break;

        int id;
        double tx, ty, tz, rx, ry, rz, var;
        int numObs = 0;

        linkbuf[0] = '\0';
        int nElems = sscanf(linebuf, "%d %lf %lf %lf %lf %lf %lf %lf %d%[^\t\n]*s", &id, &tx, &ty,
                            &tz, &rx, &ry, &rz, &var, &numObs, linkbuf);

        var += add_variance;
        if (override_variance) {
            var = add_variance;
        }

        if (nElems == 9 || nElems == 10) {
            tf2::Vector3 tvec(tx, ty, tz);
            tf2::Quaternion q;
            q.setRPY(deg2rad(rx), deg2rad(ry), deg2rad(rz));

            auto twv = TransformWithVariance(tvec, q, var);
            // TODO: figure out what the timestamp in Fiducial should be
            Fiducial f =
                Fiducial(id, tf2::Stamped<TransformWithVariance>(twv, ros::Time::now(), mapFrame));
            f.numObs = numObs;

            std::istringstream ss(linkbuf);
            std::string s;
            while (getline(ss, s, ' ')) {
                if (!s.empty()) {
                    f.links.insert(stoi(s));
                }
            }
            fiducials[id] = f;
            numRead++;
        } else {
            ROS_WARN("Invalid line: %s", linebuf);
        }
    }

    fclose(fp);
    ROS_INFO("Load map %s read %d entries", filename.c_str(), numRead);
    return true;
}

// Publish the map

void BaseMap::publishMap() {
    fiducial_msgs::FiducialMapEntryArray fmea;
    std::map<int, Fiducial>::iterator it;

    for (const auto &map_pair : fiducials) {
        const Fiducial &f = map_pair.second;

        fiducial_msgs::FiducialMapEntry fme;
        fme.fiducial_id = f.id;

        tf2::Vector3 t = f.pose.transform.getOrigin();
        fme.x = t.x();
        fme.y = t.y();
        fme.z = t.z();

        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);
        fme.rx = rx;
        fme.ry = ry;
        fme.rz = rz;

        fmea.fiducials.push_back(fme);
    }

    mapPub.publish(fmea);
}

// Publish the next marker visualization messages that hasn't been
// published recently

void BaseMap::publishMarkers() {
    ros::Time now = ros::Time::now();
    std::map<int, Fiducial>::iterator it;

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        if ((now - f.lastPublished).toSec() > 1.0) {
            publishMarker(f.id);
        }
    }
}

// Publish visualization messages for a single fiducial

void BaseMap::publishMarker(int fid) const {
    const auto &f = fiducials.at(fid); 
    auto markers = f.getMarkers(mapFrame);
    for (auto &m:markers) {
        markerPub.publish(m);
    }

    // markerPub.publish(links); TODO: add links between fiducials
}

// Publish a line marker between two points

void BaseMap::drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1) {
    static int lid = 60000;
    visualization_msgs::Marker line;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.header.frame_id = mapFrame;
    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;
    line.id = lid++;
    line.ns = "lines";
    line.scale.x = line.scale.y = line.scale.z = 0.01;
    line.pose.position.x = 0;
    line.pose.position.y = 0;
    geometry_msgs::Point gp0, gp1;
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();
    gp1.x = p1.x();
    gp1.y = p1.y();
    gp1.z = p1.z();
    line.points.push_back(gp0);
    line.points.push_back(gp1);

    markerPub.publish(line);
}

// Service to clear the map and enable auto initialization

bool BaseMap::clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Clearing fiducial map from service call");

    fiducials.clear();
    initialFrameNum = frameNum;
    originFid = -1;

    return true;
}

// Service to add a fiducial to the map

bool BaseMap::addFiducialCallback(fiducial_slam::AddFiducial::Request &req,
                              fiducial_slam::AddFiducial::Response &res)
{
   ROS_INFO("Request to add fiducial %d to map", req.fiducial_id);
   fiducialToAdd = req.fiducial_id;

   return true;
}

void BaseMap::poseCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    last_pose_variance = msg->pose.covariance[0];
}


