#pragma once

#include <list>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "transform_with_variance.h"
#include <visualization_msgs/Marker.h>

#include "Observation.h"

// A single fiducial that is in the map
class Fiducial {
public:
    int id;
    int numObs;
    bool visible;
    std::set<int> links;  // Stores the IDs of connected fiducials

    tf2::Stamped<TransformWithVariance> pose;
    ros::Time lastPublished;

    void update(const tf2::Stamped<TransformWithVariance> &newPose);

    Fiducial() {}

    Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose);

    std::vector<visualization_msgs::Marker> getMarkers(const std::string &frame_id) const;
};

