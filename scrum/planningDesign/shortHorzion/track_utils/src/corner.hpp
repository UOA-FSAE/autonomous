#pragma once

#ifndef CORNER_HPP
#define CORNER_HPP

#include <vector>
#include "DataTypes.hpp"

using namespace planning;
class Corner{

public:

    //attributes
    std::vector<Point> leftBoundary;
    std::vector<Point> rightBoundary;

    //constructor
    Corner();

    //methods
    Point getStartLeft() const;
    Point getStartRight() const;
};

#endif // CORNER_HPP