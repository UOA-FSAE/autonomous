#pragma once

#ifndef CONE_HPP
#define CONE_HPP

#include <memory>
#include <tuple>

#include <memory>
#include "DataTypes.hpp"


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

public:
    
    IntrinsicConeProp(double width) : width(width) {}
    
    virtual ~IntrinsicConeProp() {}
    
    inline double getWidth() {
        return width;
    }
    
};

class Cone {
public:
    static inline int nextId = 0;
private:

    int id;
    Point pos; // Position of the cone
    ConeType coneType; // Type of the cone
    std::shared_ptr<IntrinsicConeProp> intrinsicProps_p; // Intrinsic properties of the cone
   
public:
    Cone(Point pos, ConeType coneType, std::shared_ptr<IntrinsicConeProp> intrinsicProps): id(nextId++), pos(pos), coneType(coneType), intrinsicProps_p(intrinsicProps)  {};
    
    Cone(planning::Point point, ConeType type, IntrinsicConeProp& props) : id(nextId++), pos(point), coneType(type), intrinsicProps_p(std::make_shared<IntrinsicConeProp>(props)) {};

    Cone(planning::Point point, ConeType type) : id(nextId++), pos(point), coneType(type) {};
    
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
};
}

#endif // CONE_HPP
