#pragma once

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "transform_with_variance.h"

// An observation of a single fiducial in a single image
class Observation {
public:
    int fid;
    int t; // New: Discrete instant when the observation was made
    tf2::Stamped<TransformWithVariance> T_fidCam;
    tf2::Stamped<TransformWithVariance> T_camFid;

    Observation(){};

    Observation(int fid, const tf2::Stamped<TransformWithVariance> &camFid);

    double getDistance() const;
};
