#pragma once

#include "DataTypes.hpp"

namespace planning {

/**
 * @brief A data class for storing details about a vehicles position and 
 * 
 */
struct Vehicle {
    Point pos;
    Angle bearing; 
};


}