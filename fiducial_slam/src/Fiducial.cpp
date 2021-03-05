#include "fiducial_slam/Fiducial.h"

// Update a fiducial position in map with a new estimate
void Fiducial::update(const tf2::Stamped<TransformWithVariance> &newPose) {
    pose.update(newPose);
    numObs++;
}

// Create a fiducial from an estimate of its position in the map
Fiducial::Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose) {
    this->id = id;
    this->pose = pose;
    this->lastPublished = ros::Time(0);
    this->numObs = 0;
    this->visible = false;
}

std::vector<visualization_msgs::Marker> Fiducial::getMarkers(const std::string &frame_id) const {
    std::vector<visualization_msgs::Marker> ret_val;
    visualization_msgs::Marker marker;

    static int seq = 0;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.header.seq = ++seq;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    toMsg(pose.transform, marker.pose);

    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.01;
    if (visible) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    } else {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    }
    marker.id = id;
    marker.ns = "fiducial";
    ret_val.push_back(marker);

    // cylinder scaled by stddev
    visualization_msgs::Marker cylinder;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.header = marker.header; cylinder.header.seq = ++seq;
    cylinder.color.r = 0.0f;
    cylinder.color.g = 0.0f;
    cylinder.color.b = 1.0f;
    cylinder.color.a = 0.5f;
    cylinder.id = id + 10000;
    cylinder.ns = "sigma";
    cylinder.scale.x = cylinder.scale.y = std::max(std::sqrt(pose.variance)*0.5, 0.1);
    cylinder.scale.z = 0.01;
    cylinder.pose.position.x = marker.pose.position.x;
    cylinder.pose.position.y = marker.pose.position.y;
    cylinder.pose.position.z = marker.pose.position.z;
    cylinder.pose.position.z += (marker.scale.z / 2.0) + 0.05;
    cylinder.pose.orientation = q;
    ret_val.push_back(cylinder);

    // Text
    visualization_msgs::Marker text;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.header = marker.header; text.header.seq = ++seq;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.id = id;
    text.scale.z = 0.4;
    text.pose.position.x = marker.pose.position.x;
    text.pose.position.y = marker.pose.position.y;
    text.pose.position.z = marker.pose.position.z;
    text.pose.orientation = q;
    text.pose.position.z += (marker.scale.z / 2.0) + 0.1;
    text.id = id + 30000;
    text.ns = "text";
    text.text = std::to_string(id);
    ret_val.push_back(text);

    return ret_val;
}
