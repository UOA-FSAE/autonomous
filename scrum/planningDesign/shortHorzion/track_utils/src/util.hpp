#ifndef UTIL_HPP
#define UTIL_HPP

#include <complex>
#include <set>
#include <vector>
#include <map>
#include "DataTypes.hpp"


namespace planning {
    bool in_opposing_or_normal_direction(const Point& p1, const Point& p2, const Point& p3); //implemented and tested
    bool in_opposing_direction(const Point& p1, const Point& p2, const Point& p3); //implemented and tested
    double projection_magnitude(const Point& p1, const Point& p2, const Point& p3); //implemented and tested
    double distance(const Point& p1, const Point& p2); //implemented and tested
    double interpolate_curvature(const Point& p1, double curvature1, const Point& p2, double curvature2, const Point& p3); //implemented and tested
    
    template <typename T>
    std::vector<T> set_to_vector(const std::set<T>& set) {
        std::vector<T> vector;
        vector.reserve(set.size());

        std::copy(set.begin(), set.end(), std::back_inserter(vector));
        return vector;
    }

    template <typename K, typename T>
    std::vector<T> map_to_vector(const std::map<K, T>& map) {
        std::vector<T> vector;
        vector.reserve(map.size());

        for (const auto& pair : map) {
            vector.push_back(pair.second);
        }
        return vector;
    }
    
    template<typename T>
    std::pair<std::complex<T>, T> circle_from_3_points(std::complex<T> z1, std::complex<T> z2, std::complex<T> z3) {

        if ((z1 == z2) || (z2 == z3) || (z3 == z1)) {
            // error
        }
            
        std::complex<T> w = (z3 - z1)/(z2 - z1);
        
        // small tolerance for floating point comparisons
        if (abs(w.imag()) <= 0.000001) {
            // error
        }
        std::complex<double> j2{0,2};
        std::complex<T> c = (z2 - z1)*(w - pow(abs(w), 2))/(j2*w.imag()) + z1;
        T r = abs(z1 - c);
        
        return {c, r}; //centre of circle c ( as a complex), radius of circle r (as a real number (T))
    }

}

#endif //UTIL_HPP