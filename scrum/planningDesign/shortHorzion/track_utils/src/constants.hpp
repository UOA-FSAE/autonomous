#pragma once

namespace track_utils {
    
    const double MATCHING_THRESHOLD = 5.0;
    const double PERCEPTION_DELAY = 0.8; // cone detection and classification pipeline latency.
    const double RESPONSE_DELAY = 0.2; // planning pipeline latency microseconds

    const double CONE_SPACING = 5;
    
}