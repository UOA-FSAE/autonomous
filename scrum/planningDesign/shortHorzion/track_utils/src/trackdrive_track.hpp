#pragma once

#include "track.hpp"

using namespace planning;

class TrackdriveTrack : public Track {
public: 
    TrackdriveTrack() {};
    ~TrackdriveTrack() = default;

    std::pair<InertialPose, InertialPose> getEnd() override;
    std::pair<IntertialPose, InertialPose> getStart() override; 
}