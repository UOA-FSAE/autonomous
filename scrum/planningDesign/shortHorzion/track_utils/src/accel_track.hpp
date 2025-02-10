#pragma once

#include "track.hpp"

using namespace planning;

class AccelTrack : public Track {
public:
    AccelTrack() {};
    ~AccelTrack() = default;
    
    std::pair<InertialPose, InertialPose> getEnd() override;
    std::pair<InertialPose, InertialPose> getStart() override;
};