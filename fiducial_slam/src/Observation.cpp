#include "fiducial_slam/Observation.h"

// Constructor for observation
Observation::Observation(int fid, const tf2::Stamped<TransformWithVariance> &camFid) {
    this->fid = fid;

    T_camFid = camFid;
    T_fidCam = T_camFid;
    T_fidCam.transform = T_camFid.transform.inverse();
}

double Observation::getDistance() const {
    tf2::Vector3 t = T_camFid.transform.getOrigin();
    return sqrt(t.x() + t.y() + t.z());
}
