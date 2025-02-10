#pragma once

#include "track.hpp"

using namespace planning;

class SkidpadTrack : public Track {
public: 
    SkidpadTrack() {};
    ~SkidpadTrack() = default;
    
    std::pair<InertialPose, InertialPose> getEnd() override;
    std::pair<IntertialPose, InertialPose> getStart() override;
}
