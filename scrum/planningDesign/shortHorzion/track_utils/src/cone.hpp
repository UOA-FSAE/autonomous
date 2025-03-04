#pragma once

#ifndef CONE_HPP
#define CONE_HPP

#include <memory>
#include <tuple>
#include <map>

#include <memory>
#include "DataTypes.hpp"

// ****************** IMPORTANT ******************
//  Length units are in metres

namespace planning {

/**
 * @brief Enum for cone types
 * 
 */
enum ConeType {
    BIG_ORANGE,
    SMALL_ORANGE,
    BLUE,
    YELLOW
};


/**
 * @brief A fly-weight for storing common properties shared amongst a category
 * of instances
 * 
 */
class IntrinsicConeProp {
    
private:
    double width;
    double height;

public:
    
    IntrinsicConeProp(double width, double height) : width(width), height(height) {}
    
    virtual ~IntrinsicConeProp() {}
    
    inline double getWidth() {
        return width;
    }

    inline double getHeight() {
        return height;
    }
    
};

class Cone {
public:
    static inline int nextId = 0;
private:

    int id;
    Point pos; // Position of the cone
    ConeType coneType; // Type of the cone
   
public:
    Cone(const Point pos, const ConeType coneType): id(nextId++), pos(pos), coneType(coneType)  {};
    
    Cone(const Cone cone) : id(nextId++), pos(cone.pos), coneType(cone.conetype) {};
    
    Point getPos() const {
        return pos;
    }

    int getId() const {
        return id;
    }

    static int getNextId() {
        return nextId;
    }

    int getConeType() const {
        return coneType;
    }

    void setPos(Point newPos) {
        pos = newpos;
    }
};
}

#endif // CONE_HPP
