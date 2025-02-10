#define BOOST_TEST_MODULE My Test 
#include <boost/test/included/unit_test.hpp> 

#include "accel_track.hpp"

BOOST_AUTO_TEST_CASE(first_test)
{
  int i = 1;
  BOOST_TEST(i);
  BOOST_TEST(i == 1);
}

BOOST_AUTO_TEST_CASE(actual_test)
{
    auto prop_fly_weight_1 = IntrinsicConeProp(10);// width of 10

    auto accel_track = AccelTrack();

    std::shared_ptr<Cone> cone = std::make_shared<Cone>(Point{10,10}, BLUE, prop_fly_weight_1);

    accel_track.insertCone(cone);

    std::vector<std::shared_ptr<Cone>> retrieved_cones = accel_track.getLocalCones({10,10}, 5);

    BOOST_CHECK_EQUAL(retrieved_cones.size(), 1);

}