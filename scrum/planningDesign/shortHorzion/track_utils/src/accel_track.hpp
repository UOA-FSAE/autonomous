#pragma once

#include "track.hpp"

using namespace planning;

std::shared_ptr<IntrinsicConeProp> main_cone_prop = std::make_shared<IntrinsicConeProp>(30);

class AccelTrack : public Track {
public:
    AccelTrack() {};
    
    std::pair<Cone, Cone> getEnd() const override {

        return std::pair{Cone{{-30, 0}, BIG_ORANGE, main_cone_prop}, Cone{{30, 0}, BIG_ORANGE, main_cone_prop}};
        

    }
    std::pair<Cone, Cone> getStart() const override {
        return getEnd();
    }
};