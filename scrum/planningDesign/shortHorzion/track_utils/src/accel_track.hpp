#pragma once

#include "track.hpp"

using namespace planning;

class AccelTrack : public Track {
public:
    AccelTrack() {};
    
    std::pair<Cone, Cone> getEnd() const override {

        return std::pair{Cone{{-30, 0}, BIG_ORANGE}, Cone{{30, 0}, BIG_ORANGE}}; 
    }
    std::pair<Cone, Cone> getStart() const override {
        return getEnd();
    }
private:
    IntrinsicConeProp main_cone_prop;
};