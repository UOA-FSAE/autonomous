#define BOOST_TEST_MODULE MyTest 
#define BOOST_TEST_LOG_LEVEL message
#include <boost/test/included/unit_test.hpp> 

#include "util.hpp"
#include "track.hpp"
#include "accel_track.hpp"
#include "DataTypes.hpp"
#include "cone.hpp"
#include <memory>
#include <cmath>
#include <vector>

using namespace planning;

// Helper function to initialize a track with cones and center points
std::shared_ptr<AccelTrack> setupTrack() {
    auto track = std::make_shared<AccelTrack>();

    // Add cones to the track
    auto prop_fly_weight = std::make_shared<IntrinsicConeProp>(10); // Width of 10
    track->insertCone(Cone(Point{0, 0}, BIG_ORANGE, prop_fly_weight));
    track->insertCone(Cone(Point{10, 10}, BLUE, prop_fly_weight));
    track->insertCone(Cone(Point{20, 20}, YELLOW, prop_fly_weight));
    track->insertCone(Cone(Point{30, 30}, SMALL_ORANGE, prop_fly_weight));

    // Add center points to the track

    track->initialiseCenterPoint({
        Point{1500,1600},
        Point{1700,1650},
        Point{1800,1550},
        Point{1850,1400},
        Point{1950,1250},
        Point{2100,1200},
        Point{2250,1400},
        Point{2200,1800},
        Point{2150,2200},
        Point{2200,2500},
        Point{2300,2700},
        Point{2550,2750},
        Point{2750,2700},
        Point{2800,2500},
        Point{2650,2250},
        Point{2650,2050},
        Point{2750,1750},
        Point{3000,1650},
        Point{3200,1700},
        Point{3300,1950},
        Point{3250,2200},
        Point{3200,2600},
        Point{3100,2800},
        Point{2900,3000},
        Point{2700,3200},
        Point{2400,3300},
        Point{2000,3300},
        Point{1700,3250},
        Point{1500,3300},
        Point{1250,3250},
        Point{1150,3000},
        Point{1150,2800},
        Point{1200,2600},
        Point{1150,2300},
        Point{1200,1950},
        Point{1200,1650}
    }, true);
        
    return track;
}

const double THRESHOLD = 0.00001;

BOOST_AUTO_TEST_SUITE()

BOOST_AUTO_TEST_CASE(first_test)
{
  using namespace planning;
  int i = 1;
  BOOST_TEST(i);
  BOOST_TEST(i == 1);
}



BOOST_AUTO_TEST_CASE(actual_test)
{
  using namespace planning;
  auto prop_fly_weight_1 = IntrinsicConeProp(10);// width of 10

  auto accel_track = AccelTrack();

  Cone cone = Cone(Point{10,10}, BLUE, prop_fly_weight_1);

  accel_track.insertCone(cone);

    std::vector<Cone> retrieved_cones = accel_track.getLocalCones({10,10}, 5);;

  BOOST_CHECK_EQUAL(retrieved_cones.size(), 1);

}

BOOST_AUTO_TEST_CASE(test_in_opposing_or_normal_direction) {
    using namespace planning;

    // Case 1: Queried point is behind the line segment but fully along the same line
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(-1, 0);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 1: Queried point is behind the line segment and colinear");
        BOOST_TEST_MESSAGE("Expected: true, Actual: " << std::boolalpha << result);
        BOOST_CHECK(result);
    }

    // Case 2: Queried point is in front of the line segment but fully along the same line
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(2, 0);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 2: Queried point is in front of the line segment and colinear");
        BOOST_TEST_MESSAGE("Expected: false, Actual: " << std::boolalpha << result);
        BOOST_CHECK(!result);
    }

    // Case 3: Queried point is on the line segment but between teh other two points
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(0.5, 0);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 3: Queried point is on the line segment but between the other two points");
        BOOST_TEST_MESSAGE("Expected: false, Actual: " << std::boolalpha << result);
        BOOST_CHECK(!result);
    }

    // Case 4: Queried point is on the line segment with magnitude less than the vector of the other points
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(-0.5, 0);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 3: Queried point is on the line segment bwith magnitude less than the vector of the other point");
        BOOST_TEST_MESSAGE("Expected: true, Actual: " << std::boolalpha << result);
        BOOST_CHECK(result);
    }

    // Case 5: Queried point is perpendicular to the line segment
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(0, 1);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 5: Queried point is perpendicular to the line segment");
        BOOST_TEST_MESSAGE("Expected: true, Actual: " << std::boolalpha << result);
        BOOST_CHECK(result);
    }

    // Case 6: Queried point is at the common vertex
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(0, 0);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 6: Queried point is at the common vertex");
        BOOST_TEST_MESSAGE("Expected: true, Actual: " << std::boolalpha << result);
        BOOST_CHECK(result);
    }

    //Case 7 : Queried point is in front of the common vertex but not co-linear
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(0.3, 1.2);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 7: Queried point is in front of the common vertex but not co-linear");
        BOOST_TEST_MESSAGE("Expected: false, Actual: " << std::boolalpha << result);
        BOOST_CHECK(!result);
    }

    //Case 8 : Queried point is behind the common vertex but not co-linear
    {
        Point common_vertex(0, 0);
        Point projection_head(1, 0);
        Point queried_head(-0.6, 0.45);
        bool result = in_opposing_or_normal_direction(common_vertex, projection_head, queried_head);
        BOOST_TEST_MESSAGE("Test Case 8: Queried point is in front of the common vertex but not co-linear");
        BOOST_TEST_MESSAGE("Expected: true, Actual: " << std::boolalpha << result);
        BOOST_CHECK(result);
    }

}

BOOST_AUTO_TEST_CASE(test_projection_magnitude)
{
  using namespace planning;
  // Case 1: Normal projection
  Point a(0, 0);
  Point b(4, 0);
  Point c(2, 3);
  BOOST_CHECK_CLOSE(projection_magnitude(a, b, c), 2.0, 1e-6);
    
  // Case 2: Point C is on AB
  Point d(3, 0);
  BOOST_CHECK_CLOSE(projection_magnitude(a, b, d), 3.0, 1e-6);
    
  // Case 3: A and B are the same (degenerate case)
  Point e(0, 0);
  BOOST_CHECK_EQUAL(projection_magnitude(a, e, c), 0.0);
    
  // Case 4: Projection is negative (C before A)
  Point f(-2, 1);
  BOOST_CHECK_CLOSE(projection_magnitude(a, b, f), -2.0, 1e-6);
    
  //Case 5: B comes before A (reverse vector case)
  BOOST_CHECK_CLOSE(projection_magnitude(b, a, c), 2.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(test_interpolate_curvature)
{
    using namespace planning;
    
    // Helper function to print test case details
    auto print_test_case = [](int case_num, const std::string& description, double expected, double actual) {
        BOOST_TEST_MESSAGE("Test Case " << case_num << ": " << description);
        BOOST_TEST_MESSAGE("Expected: " << expected << ", Actual: " << actual);
    };
    
    // Case 1: Simple linear interpolation - midpoint
    {
        Point A(0, 0);
        double kappa_A = 1.0;
        Point B(10, 0);
        double kappa_B = 2.0;
        Point C(5, 0); // Midpoint
        
        double expected = 1.5; // Halfway between 1.0 and 2.0
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(1, "Simple linear interpolation - midpoint", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
    
    // Case 2: Simple linear interpolation - 75% of the way
    {
        Point A(0, 0);
        double kappa_A = 1.0;
        Point B(10, 0);
        double kappa_B = 2.0;
        Point C(7.5, 0); // 75% of the way from A to B
        
        double expected = 1.75; // 75% of the way from 1.0 to 2.0
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(2, "Simple linear interpolation - 75% of the way", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
    
    // Case 3: Interpolation with different curvature signs
    {
        Point A(0, 0);
        double kappa_A = -1.0;
        Point B(10, 0);
        double kappa_B = 1.0;
        Point C(5, 0); // Midpoint
        
        double expected = 0.0; // Halfway between -1.0 and 1.0
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(3, "Interpolation with different curvature signs", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
    
    // Case 4: Interpolation with extreme curvature values
    {
        Point A(0, 0);
        double kappa_A = 0.0;
        Point B(10, 0);
        double kappa_B = 100.0;
        Point C(2, 0); // 20% of the way from A to B
        
        double expected = 20.0; // 20% of the way from 0.0 to 100.0
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(4, "Interpolation with extreme curvature values", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
    
    // Case 5: Interpolation in 2D space
    {
        Point A(0, 0);
        double kappa_A = 1.0;
        Point B(10, 10);
        double kappa_B = 3.0;
        Point C(5, 5); // Midpoint in 2D space
        
        double expected = 2.0; // Halfway between 1.0 and 3.0
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(5, "Interpolation in 2D space", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
    
    // Case 6: Interpolation beyond point B
    {
        Point A(0, 0);
        double kappa_A = 1.0;
        Point B(10, 0);
        double kappa_B = 2.0;
        Point C(15, 0); // Beyond point B
        
        double expected = 2.5; // Extrapolated value
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(6, "Interpolation beyond point B", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
    
    // Case 7: Interpolation before point A
    {
        Point A(0, 0);
        double kappa_A = 1.0;
        Point B(10, 0);
        double kappa_B = 2.0;
        Point C(-5, 0); // Before point A
        
        double expected = 0.5; // Extrapolated value
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(7, "Interpolation before point A", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
    
    // Case 8: Special case - A and B are the same point
    {
        Point A(0, 0);
        double kappa_A = 1.0;
        Point B(0, 0); // Same as A
        double kappa_B = 2.0;
        Point C(5, 0);
        
        // When A and B are the same, the function should return kappa_A
        double expected = kappa_A;
        double result = interpolate_curvature(A, kappa_A, B, kappa_B, C);
        print_test_case(8, "Special case - A and B are the same point", expected, result);
        BOOST_CHECK_CLOSE(result, expected, THRESHOLD);
    }
}

BOOST_AUTO_TEST_CASE(test_getLocalCones) {
    auto track = setupTrack();

    // Test case 1: Retrieve cones within range of (10, 10)
    std::vector<Cone> cones1 = track->getLocalCones(Point{10, 10}, 15); // Range of 15
    BOOST_CHECK_EQUAL(cones1.size(), 3); // Expect 3 cones: (0, 0), (10, 10), and (20, 20)
    
    // Verify the positions of retrieved cones
    std::vector<Point> expected_positions1 = {Point{0, 0}, Point{10, 10}, Point{20, 20}};
    for (const auto& cone : cones1) {
        bool found = false;
        for (const auto& expected_pos : expected_positions1) {
            if (cone.getPos().distanceTo(expected_pos) < THRESHOLD) {
                found = true;
                break;
            }
        }
        BOOST_CHECK(found);
    }

    // Test case 2: Retrieve cones within range of (25, 25)
    std::vector<Cone> cones2 = track->getLocalCones(Point{25, 25}, 10); // Range of 10
    BOOST_CHECK_EQUAL(cones2.size(), 2); // Expect 2 cones: (20, 20) and (30, 30)
    
    // Verify the positions of retrieved cones
    std::vector<Point> expected_positions2 = {Point{20, 20}, Point{30, 30}};
    for (const auto& cone : cones2) {
        bool found = false;
        for (const auto& expected_pos : expected_positions2) {
            if (cone.getPos().distanceTo(expected_pos) < THRESHOLD) {
                found = true;
                break;
            }
        }
        BOOST_CHECK(found);
    }

    // Test case 3: Retrieve cones within range of (0, 0)
    std::vector<Cone> cones3 = track->getLocalCones(Point{0, 0}, 5); // Range of 5
    BOOST_CHECK_EQUAL(cones3.size(), 1); // Expect 1 cone: (0, 0)
    BOOST_CHECK(cones3[0].getPos().distanceTo(Point{0, 0}) < THRESHOLD);

    // Test case 4: No cones within range
    std::vector<Cone> cones4 = track->getLocalCones(Point{100, 100}, 10); // Range of 10
    BOOST_CHECK_EQUAL(cones4.size(), 0); // Expect no cones
}


BOOST_AUTO_TEST_CASE(test_getLocalCenterPoints) {
    auto track = setupTrack();

    // Test case 1: Point exactly on a center point
    Point testPoint1{2750, 1750};  // This point exists in our centerPoints
    auto centerPoints1 = track->getLocalCenterPoints(testPoint1, 10);
    BOOST_CHECK_EQUAL(centerPoints1.size(), 1);
    BOOST_TEST_MESSAGE("Test case 1: Point exactly on center point (2750, 1750):");
    for (const auto& point : centerPoints1) {
        BOOST_TEST_MESSAGE("Found point: (" << point.pos.x << ", " << point.pos.y << ")");
    }

    // Test case 2: Point between two center points with appropriate range
    Point testPoint2{2875, 1700};  // Between (2750,1750) and (3000,1650)
    auto centerPoints2 = track->getLocalCenterPoints(testPoint2, 200);  // Range covers both points
    BOOST_CHECK_EQUAL(centerPoints2.size(), 2);
    BOOST_TEST_MESSAGE("Test case 2: Point between center points (2875, 1700):");
    for (const auto& point : centerPoints2) {
        BOOST_TEST_MESSAGE("Found point: (" << point.pos.x << ", " << point.pos.y << ")");
    }

    // Test case 3: Point near track with small range (should find one point)
    Point testPoint3{2760, 1760};  // Very close to (2750,1750)
    auto centerPoints3 = track->getLocalCenterPoints(testPoint3, 20);
    BOOST_CHECK_EQUAL(centerPoints3.size(), 1);
    BOOST_TEST_MESSAGE("Test case 3: Point near track (2760, 1760):");
    for (const auto& point : centerPoints3) {
        BOOST_TEST_MESSAGE("Found point: (" << point.pos.x << ", " << point.pos.y << ")");
    }

    // Test case 4: Point far from track (should find no points)
    Point testPoint4{5000, 5000};
    auto centerPoints4 = track->getLocalCenterPoints(testPoint4, 100);
    BOOST_CHECK_EQUAL(centerPoints4.size(), 0);
    BOOST_TEST_MESSAGE("Test case 4: Point far from track (5000, 5000):");
    BOOST_TEST_MESSAGE("Found " << centerPoints4.size() << " points (expected 0)");

    // Test case 5: Point with large range (should find multiple points)
    Point testPoint5{2750, 1750};
    auto centerPoints5 = track->getLocalCenterPoints(testPoint5, 500);
    BOOST_CHECK(centerPoints5.size() > 2);  // Should find several points
    BOOST_TEST_MESSAGE("Test case 5: Point with large range (2750, 1750):");
    BOOST_TEST_MESSAGE("Found " << centerPoints5.size() << " points");
    for (const auto& point : centerPoints5) {
        BOOST_TEST_MESSAGE("Found point: (" << point.pos.x << ", " << point.pos.y << ")");
    }
}

BOOST_AUTO_TEST_CASE(test_getNearestCenterPoints) {
    // Helper function to format optional points for output
    auto format_optional_point = [](const std::optional<InertialPose>& opt) -> std::string {
        if (opt.has_value()) {
            return "Point(" + std::to_string(opt.value().pos.x) + ", " + std::to_string(opt.value().pos.y) + ")";
        } else {
            return "nullopt";
        }
    };

    // Case 1: Test empty track
    {
        auto emptyTrack = std::make_shared<AccelTrack>();
        auto empty_result = emptyTrack->getNearestCenterPoints(Point{0, 0});
        BOOST_TEST_MESSAGE("Case 1: Test empty track");
        BOOST_TEST_MESSAGE("Expected: {nullopt, nullopt}, Actual: {" 
                           << format_optional_point(empty_result.first) << ", " 
                           << format_optional_point(empty_result.second) << "}");
        BOOST_CHECK(!empty_result.first.has_value());
        BOOST_CHECK(!empty_result.second.has_value());
    }

    // Create a track with three points
    auto track = std::make_shared<AccelTrack>();
    std::vector<Point> threePoints = {
        Point{1500, 1600}, 
        Point{1700, 1650}, 
        Point{1800, 1550}
    };
    track->initialiseCenterPoint(std::move(threePoints), false);

    // Case 3: Open loop, queried point before first center point
    {
        auto before_start = track->getNearestCenterPoints(Point{1400, 1550});
        BOOST_TEST_MESSAGE("Case 3: Open loop, queried point before first center point");
        BOOST_TEST_MESSAGE("Expected: {nullopt, Point(1500, 1600)}, Actual: {" 
                           << format_optional_point(before_start.first) << ", " 
                           << format_optional_point(before_start.second) << "}");
        BOOST_CHECK(!before_start.first.has_value());
        BOOST_CHECK(before_start.second.has_value());
        if (before_start.second.has_value()) {  // Add safety check
            BOOST_CHECK_EQUAL(before_start.second.value().pos.x, 1500);
        }
    }

    // Case 4: Open loop, queried point between first and second center points
    {
        auto between_first_second = track->getNearestCenterPoints(Point{1600, 1625});
        BOOST_TEST_MESSAGE("Case 4: Open loop, queried point between first and second center points");
        BOOST_TEST_MESSAGE("Expected: {Point(1500, 1600), Point(1700, 1650)}, Actual: {" 
                           << format_optional_point(between_first_second.first) << ", " 
                           << format_optional_point(between_first_second.second) << "}");
        BOOST_CHECK(between_first_second.first.has_value());
        BOOST_CHECK(between_first_second.second.has_value());
        BOOST_CHECK_EQUAL(between_first_second.first.value().pos.x, 1500);
        BOOST_CHECK_EQUAL(between_first_second.second.value().pos.x, 1700);
    }

    // Case 5: Open loop, queried point between second and third center points
    {
        auto between_second_third = track->getNearestCenterPoints(Point{1750, 1600});
        BOOST_TEST_MESSAGE("Case 5: Open loop, queried point between second and third center points");
        BOOST_TEST_MESSAGE("Expected: {Point(1700, 1650), Point(1800, 1550)}, Actual: {" 
                           << format_optional_point(between_second_third.first) << ", " 
                           << format_optional_point(between_second_third.second) << "}");
        BOOST_CHECK(between_second_third.first.has_value());
        BOOST_CHECK(between_second_third.second.has_value());
        BOOST_CHECK_EQUAL(between_second_third.first.value().pos.x, 1700);
        BOOST_CHECK_EQUAL(between_second_third.second.value().pos.x, 1800);
    }

    // Case 6: Open loop, queried point after last center point
    {
        auto after_end = track->getNearestCenterPoints(Point{1850, 1500});
        BOOST_TEST_MESSAGE("Case 6: Open loop, queried point after last center point");
        BOOST_TEST_MESSAGE("Expected: {Point(1800, 1550), nullopt}, Actual: {" 
                           << format_optional_point(after_end.first) << ", " 
                           << format_optional_point(after_end.second) << "}");
        BOOST_CHECK(after_end.first.has_value());
        BOOST_CHECK(!after_end.second.has_value());
        BOOST_CHECK_EQUAL(after_end.first.value().pos.x, 1800);
    }

    // Test closed loop cases
    track->setClosedLoop();

    // Case 7: Closed loop, queried point between last and first center points
    {
        auto between_last_first = track->getNearestCenterPoints(Point{1400, 1550});
        BOOST_TEST_MESSAGE("Case 7: Closed loop, queried point between last and first center points");
        BOOST_TEST_MESSAGE("Expected: {Point(1800, 1550), Point(1500, 1600)}, Actual: {" 
                           << format_optional_point(between_last_first.first) << ", " 
                           << format_optional_point(between_last_first.second) << "}");
        BOOST_CHECK(between_last_first.first.has_value());
        BOOST_CHECK(between_last_first.second.has_value());
        BOOST_CHECK_EQUAL(between_last_first.first.value().pos.x, 1800);
        BOOST_CHECK_EQUAL(between_last_first.second.value().pos.x, 1500);
    }

    // Case 8: Closed loop, queried point between first and second center points
    {
        auto closed_between_first_second = track->getNearestCenterPoints(Point{1600, 1625});
        BOOST_TEST_MESSAGE("Case 8: Closed loop, queried point between first and second center points");
        BOOST_TEST_MESSAGE("Expected: {Point(1500, 1600), Point(1700, 1650)}, Actual: {" 
                           << format_optional_point(closed_between_first_second.first) << ", " 
                           << format_optional_point(closed_between_first_second.second) << "}");
        BOOST_CHECK(closed_between_first_second.first.has_value());
        BOOST_CHECK(closed_between_first_second.second.has_value());
        BOOST_CHECK_EQUAL(closed_between_first_second.first.value().pos.x, 1500);
        BOOST_CHECK_EQUAL(closed_between_first_second.second.value().pos.x, 1700);
    }
}

BOOST_AUTO_TEST_CASE(test_getCurvature_with_tuple_points)
{
  using namespace planning;

  Point a(10, 10);
  Point b(10,11);
  Point c(11,11);
  
  AccelTrack aTrack{};
  
  auto res = aTrack.getCurvature({a,b,c});
  double expected = 1.41421;

  // getCurvature
  BOOST_CHECK(res - expected <= THRESHOLD);

}

BOOST_AUTO_TEST_SUITE_END()