// #include <iostream>
// #include <vector>
// #include <string>
// #include <set>
// #include <map>
// #include <cmath>   // for std::sqrt, std::hypot
// #include <utility>

// #include "cone.hpp"
// #include "track.hpp"
// #include "util.hpp"
// #include "DataTypes.hpp"

// // ------------ Begin CGAL Includes ------------
// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Delaunay_triangulation_2.h>
// // ---------------------------------------------

// // Define a CGAL Kernel
// typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
// typedef K::Point_2 Point;



// std::vector<planning::InertialPose> triangulateCenterPoints(std::vector<planning::Cone> &cones) {
//     // 1) Build the Delaunay triangulation
//     Delaunay dt;
//     // Map each CGAL Vertex_handle to the cone's colour
//     std::map<Delaunay::Vertex_handle, int> color_map;

//     for (auto &cone : cones) {
//         Delaunay::Vertex_handle vh = dt.insert(Point(cone.getPos().x, cone.getPos().y));
//         color_map[vh] = cone.getConeType();
//     }

//     // 2) Identify valid faces (triangles) that have more than one color among their vertices
//     std::vector<Delaunay::Face_handle> valid_faces;
//     for (auto f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f) {
//         // Collect the colours of the triangle's vertices
//         std::set<int> face_colors;
//         for (int i = 0; i < 3; ++i) {
//             face_colors.insert(color_map[f->vertex(i)]);
//         }
//         // If more than one colour is present, keep it
//         if (face_colors.size() > 1) {
//             valid_faces.push_back(f);
//         }
//     }

//     // 3) Collect unique midpoints of edges that connect differently-colored cones
//     // duplicates are due to same edges being shared between two triangles
//     std::set<planning::InertialPose> midpoint_set;

//     for (auto &face : valid_faces) {
//         // Each face has 3 edges: (v0,v1), (v1,v2), (v2,v0)
//         for (int i = 0; i < 3; ++i) {
//             Delaunay::Vertex_handle vh1 = face->vertex(i);
//             Delaunay::Vertex_handle vh2 = face->vertex((i+1) % 3);

//             if (color_map[vh1] != color_map[vh2]) {
//                 Point p1 = vh1->point();
//                 Point p2 = vh2->point();
//                 double mx = (p1.x() + p2.x()) / 2.0;
//                 double my = (p1.y() + p2.y()) / 2.0;
//                 midpoint_set.insert(planning::InertialPose(planning::Point(mx, my)));
//             }
//         }
//     }
//     auto center_points_vector = planning::set_to_vector<planning::InertialPose>(midpoint_set);

//     return center_points_vector;
// }


// // int main() {
// //     // 1) Define your cone data (just as in Python)
// //     std::vector<Cone> cones = {
// //         {  2.0,  0.0, "b" },
// //         { -1.0,  2.0, "y" },
// //         {  3.0,  2.0, "b" },
// //         {  4.5,  3.5, "b" },
// //         {  0.5,  3.5, "y" },
// //         { -2.0, -2.0, "y" },
// //         {  2.0, -2.0, "b" },
// //         { -2.0, -4.0, "y" },
// //         {  2.0, -4.0, "b" },
// //         { -2.0,  0.0, "y" }
// //     };

// //     // 2) Call the function that does all the triangulation & midpoint logic
// //     std::set<std::pair<double,double>> center_points = triangulateCenterPoints(cones);

// //     // 3) Print the results
// //     std::cout << "Unique center points (midpoints of differently-colored edges):\n";
// //     for (auto &pt : center_points) {
// //         std::cout << "(" << pt.first << ", " << pt.second << ")\n";
// //     }

// //     //example using matchCentrePoints
// //     std::pair<double,double> candidate = {1.0, 1.0};
// //     auto match = matchCenterPoints(center_points, candidate, 1.0);
// //     std::cout << "Candidate (1.0, 1.0) matched or inserted => ("
// //               << match.first << ", " << match.second << ")\n";
// //     return 0;
// // }
