#include "corner.hpp"
#include <stdexcept>

Point Corner::getStartLeft() const{
    if(!leftBoundary.empty()){
        return leftBoundary.front();
    } else{
        throw std::runtime_error("Left Boundary is empty");
    }
} 

Point Corner::getStartRight() const{
    if(!rightBoundary.empty()){
        return rightBoundary.front();
    } else{
        throw std::runtime_error("Right Boundary is empty");
    }
} 



