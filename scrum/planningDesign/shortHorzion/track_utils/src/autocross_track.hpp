#pragma once

#include "track.hpp"

using namespace planning;

class AutoCrossTrack : public Track {
public:
    AutoCrossTrack() {};
    ~AutoCrossTrack() = default;

    std::pair<IntertialPose, InertialPose> getStart() override;
    std::pair<InertialPose, InertialPose> getEnd() override {
        return {};
    }
};