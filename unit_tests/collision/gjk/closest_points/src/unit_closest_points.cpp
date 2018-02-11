//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_voronoi_simplex_solver_policy.h>
#include <OpenTissue/collision/gjk/gjk_compute_closest_points.h>
#include <OpenTissue/collision/gjk/gjk_support_functors.h>



#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_compute_closest_points);


BOOST_AUTO_TEST_CASE(case_by_case_testing)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;

  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef math_types::coordsys_type                        transformation_type;
  typedef math_types::value_traits                         value_traits;

  OpenTissue::gjk::VoronoiSimplexSolverPolicy const simplex_solver_policy = OpenTissue::gjk::VoronoiSimplexSolverPolicy();

  OpenTissue::gjk::Sphere<math_types> const supportA;
  OpenTissue::gjk::Sphere<math_types> const supportB;

  size_t    const max_iterations       = 100u;
  real_type const absolute_tolerance   = boost::numeric_cast<real_type>(10e-6);
  real_type const relative_tolerance   = boost::numeric_cast<real_type>(10e-6);
  real_type const stagnation_tolerance = boost::numeric_cast<real_type>(10e-15);

  // Two unit-spheres placed ontop of each other
  {
    transformation_type transformA;
    transformation_type transformB;

    vector3_type a;
    vector3_type b;
    size_t iterations     = 0u;
    size_t status         = 0u;
    real_type distance    = value_traits::infinity();


    transformA.T().clear();
    transformA.Q().identity();
    transformB.T().clear();
    transformB.Q().identity();

    OpenTissue::gjk::compute_closest_points(
      transformA
      , supportA
      , transformB
      , supportB
      , a
      , b
      , distance
      , iterations
      , status
      , absolute_tolerance
      , relative_tolerance
      , stagnation_tolerance
      , max_iterations
      , simplex_solver_policy
      );


    real_type true_distance = length( transformA.T() - transformB.T() ) - value_traits::two();

    std::cout << "\tstatus = " << OpenTissue::gjk::get_status_message(status) << std::endl;
    std::cout << "\tdistance = " << distance << std::endl;
    std::cout << "\titerations = " << iterations << std::endl;
    std::cout << "\ttrue distance = " << true_distance << std::endl;

  }
  // Two unit-spheres overlapping but both placed on the x-axis
  {
    transformation_type transformA;
    transformation_type transformB;

    vector3_type a;
    vector3_type b;
    size_t iterations     = 0u;
    size_t status         = 0u;
    real_type distance    = value_traits::infinity();


    transformA.T().clear();
    transformA.Q().identity();
    transformA.T()(0) = -1.5;
    transformB.T().clear();
    transformB.Q().identity();

    OpenTissue::gjk::compute_closest_points(
      transformA
      , supportA
      , transformB
      , supportB
      , a
      , b
      , distance
      , iterations
      , status
      , absolute_tolerance
      , relative_tolerance
      , stagnation_tolerance
      , max_iterations
      , simplex_solver_policy
      );

    real_type true_distance = length( transformA.T() - transformB.T() ) - value_traits::two();

    std::cout << "\tstatus = " << OpenTissue::gjk::get_status_message(status) << std::endl;
    std::cout << "\tdistance = " << distance << std::endl;
    std::cout << "\titerations = " << iterations << std::endl;
    std::cout << "\ttrue distance = " << true_distance << std::endl;
  }
  // Two unit-spheres exactly touching in one point (= one intersection point) and but both placed on the x-axis
  {
    transformation_type transformA;
    transformation_type transformB;

    vector3_type a;
    vector3_type b;
    size_t iterations     = 0u;
    size_t status         = 0u;
    real_type distance    = value_traits::infinity();


    transformA.T().clear();
    transformA.Q().identity();
    transformA.T()(0) = -2.0;
    transformB.T().clear();
    transformB.Q().identity();

    OpenTissue::gjk::compute_closest_points(
      transformA
      , supportA
      , transformB
      , supportB
      , a
      , b
      , distance
      , iterations
      , status
      , absolute_tolerance
      , relative_tolerance
      , stagnation_tolerance
      , max_iterations
      , simplex_solver_policy
      );


    real_type true_distance = length( transformA.T() - transformB.T() ) - value_traits::two();

    std::cout << "\tstatus = " << OpenTissue::gjk::get_status_message(status) << std::endl;
    std::cout << "\tdistance = " << distance << std::endl;
    std::cout << "\titerations = " << iterations << std::endl;
    std::cout << "\ttrue distance = " << true_distance << std::endl;

  }
  // Two unit-spheres non-overlapping but both placed on the x-axis
  {
    transformation_type transformA;
    transformation_type transformB;

    vector3_type a;
    vector3_type b;
    size_t iterations     = 0u;
    size_t status         = 0u;
    real_type distance    = value_traits::infinity();


    transformA.T().clear();
    transformA.Q().identity();
    transformA.T()(0) = -2.5;
    transformB.T().clear();
    transformB.Q().identity();

    OpenTissue::gjk::compute_closest_points(
      transformA
      , supportA
      , transformB
      , supportB
      , a
      , b
      , distance
      , iterations
      , status
      , absolute_tolerance
      , relative_tolerance
      , stagnation_tolerance
      , max_iterations
      , simplex_solver_policy
      );


    real_type true_distance = length( transformA.T() - transformB.T() ) - value_traits::two();

    std::cout << "\tstatus = " << OpenTissue::gjk::get_status_message(status) << std::endl;
    std::cout << "\tdistance = " << distance << std::endl;
    std::cout << "\titerations = " << iterations << std::endl;
    std::cout << "\ttrue distance = " << true_distance << std::endl;
  }



}



BOOST_AUTO_TEST_CASE(random_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;

  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef math_types::coordsys_type                        transformation_type;
  typedef math_types::value_traits                         value_traits;

  typedef OpenTissue::gjk::Simplex<vector3_type>           simplex_type;


  OpenTissue::gjk::VoronoiSimplexSolverPolicy const simplex_solver_policy = OpenTissue::gjk::VoronoiSimplexSolverPolicy();

  OpenTissue::gjk::Sphere<math_types> const supportA;
  OpenTissue::gjk::Sphere<math_types> const supportB;

  size_t    const max_iterations       = 100u;
  real_type const absolute_tolerance   = boost::numeric_cast<real_type>(10e-6);
  real_type const relative_tolerance   = boost::numeric_cast<real_type>(10e-10);
  real_type const stagnation_tolerance = boost::numeric_cast<real_type>(0.0);

  for(size_t i=0;i<100u;++i)
  {
    transformation_type transformA;
    transformation_type transformB;

    quaternion_type Q;
    vector3_type a;
    vector3_type b;
    size_t iterations     = 0u;
    size_t status         = 0u;
    real_type distance    = value_traits::infinity();


    OpenTissue::math::random( transformA.T(), -2.0, 2.0 );
    OpenTissue::math::random( transformB.T(), -2.0, 2.0 );
    Q.random();
    transformA.Q() = OpenTissue::math::unit ( Q );
    Q.random();
    transformB.Q() = OpenTissue::math::unit ( Q );

    OpenTissue::gjk::compute_closest_points(
      transformA
      , supportA
      , transformB
      , supportB
      , a
      , b
      , distance
      , iterations
      , status
      , absolute_tolerance
      , relative_tolerance
      , stagnation_tolerance
      , max_iterations
      , simplex_solver_policy
      );

    real_type true_distance = length( transformA.T() - transformB.T() ) - value_traits::two();

    if(  true_distance > absolute_tolerance )
    {
      BOOST_CHECK_CLOSE( true_distance, distance, 0.05 );

      BOOST_CHECK( status != OpenTissue::gjk::ABSOLUTE_CONVERGENCE );            // Would indicate penetration
      BOOST_CHECK( status != OpenTissue::gjk::INTERSECTION );                    // Would indicate penetration
      BOOST_CHECK( status != OpenTissue::gjk::ITERATING );                       // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::EXCEEDED_MAX_ITERATIONS_LIMIT );   // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::NON_DESCEND_DIRECTION );           // Would indicate internal error in GJK
      //BOOST_CHECK( status != OpenTissue::gjk::RELATIVE_CONVERGENCE );          // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::SIMPLEX_EXPANSION_FAILED );      // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::STAGNATION );                    // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::LOWER_ERROR_BOUND_CONVERGENCE ); // Would indicate convergence to positive distance
    }
    else
    {
      BOOST_CHECK( 0.0 <= distance );
      BOOST_CHECK( distance < absolute_tolerance );

      BOOST_CHECK( status != OpenTissue::gjk::STAGNATION );                     // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::LOWER_ERROR_BOUND_CONVERGENCE );  // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::RELATIVE_CONVERGENCE );           // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::SIMPLEX_EXPANSION_FAILED );       // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::ITERATING );                      // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::EXCEEDED_MAX_ITERATIONS_LIMIT );  // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::NON_DESCEND_DIRECTION );          // Would indicate internal error in GJK
      //BOOST_CHECK( status != OpenTissue::gjk::ABSOLUTE_CONVERGENCE );         // Indicates penetration
      //BOOST_CHECK( status != OpenTissue::gjk::INTERSECTION );                 // Indicates penetration
    }

    OpenTissue::gjk::compute_closest_points(
      transformA
      , supportA
      , transformB
      , supportB
      , a
      , b
      , distance
      , status
      );

    if(  true_distance > absolute_tolerance )
    {
      BOOST_CHECK_CLOSE( true_distance, distance, 0.05 );

      BOOST_CHECK( status != OpenTissue::gjk::ABSOLUTE_CONVERGENCE );            // Would indicate penetration
      BOOST_CHECK( status != OpenTissue::gjk::INTERSECTION );                    // Would indicate penetration
      BOOST_CHECK( status != OpenTissue::gjk::ITERATING );                       // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::EXCEEDED_MAX_ITERATIONS_LIMIT );   // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::NON_DESCEND_DIRECTION );           // Would indicate internal error in GJK
      //BOOST_CHECK( status != OpenTissue::gjk::RELATIVE_CONVERGENCE );          // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::SIMPLEX_EXPANSION_FAILED );      // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::STAGNATION );                    // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::LOWER_ERROR_BOUND_CONVERGENCE ); // Would indicate convergence to positive distance
    }
    else
    {
      BOOST_CHECK( 0.0 <= distance );
      BOOST_CHECK( distance < absolute_tolerance );

      BOOST_CHECK( status != OpenTissue::gjk::STAGNATION );                     // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::LOWER_ERROR_BOUND_CONVERGENCE );  // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::RELATIVE_CONVERGENCE );           // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::SIMPLEX_EXPANSION_FAILED );       // Can only occur in case of positive distance
      BOOST_CHECK( status != OpenTissue::gjk::ITERATING );                      // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::EXCEEDED_MAX_ITERATIONS_LIMIT );  // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::NON_DESCEND_DIRECTION );          // Would indicate internal error in GJK
      //BOOST_CHECK( status != OpenTissue::gjk::ABSOLUTE_CONVERGENCE );         // Indicates penetration
      //BOOST_CHECK( status != OpenTissue::gjk::INTERSECTION );                 // Indicates penetration
    }
  }
}



BOOST_AUTO_TEST_CASE(cylinder_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;

  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef math_types::coordsys_type                        transformation_type;
  typedef math_types::value_traits                         value_traits;

  typedef OpenTissue::gjk::Simplex<vector3_type>           simplex_type;

  OpenTissue::gjk::VoronoiSimplexSolverPolicy const simplex_solver_policy = OpenTissue::gjk::VoronoiSimplexSolverPolicy();

  // We let the cylinder move around the sphere in a fixed distance. Thus
  // we always know the closest point by construction.
  OpenTissue::gjk::Sphere<math_types> const supportA;
  OpenTissue::gjk::Cylinder<math_types> const supportB;

  size_t    const max_iterations       = 1000u;
  real_type const absolute_tolerance   = boost::numeric_cast<real_type>(0.0);
  real_type const relative_tolerance   = boost::numeric_cast<real_type>(10e-10); // Don't be to over agressive!
  real_type const stagnation_tolerance = boost::numeric_cast<real_type>(0.0);

  real_type rad_x = value_traits::zero();
  real_type rad_z = value_traits::zero();

  for(size_t i=0;i<100u;++i)
  {
    transformation_type transformA;
    transformation_type transformB;

    quaternion_type Qx,Qz;
    vector3_type a;
    vector3_type b;
    size_t iterations     = 0u;
    size_t status         = 0u;
    real_type distance    = value_traits::infinity();

    // Place sphere at origin
    transformA.T().clear();
    transformA.Q().identity();

    // Place cylinder a fixed distance away from sphere
    transformB.T().clear();
    transformB.Q().identity();

    // Minimum distance between sphere and cylinder is fixed by design

    // The unrotated cloest points
    vector3_type tst_a = vector3_type(1.0, 0.0, 0.0);
    vector3_type tst_b = vector3_type(1.1, 0.0, 0.0);

    // Apply some rotation to the cylinder, which does not alter the closest points
    Qx.Rx( rad_x );
    Qz.Rz( rad_z );
    rad_x += value_traits::pi()/100.0;
    rad_z += value_traits::pi()/100.0;

    transformB.Q() = prod( Qz, prod( Qx , transformB.Q() ));
    transformB.T() = Qz.rotate( vector3_type( 2.1, 0.0, 0.0 ) );

    // Update closest points with rotation
    tst_a = Qz.rotate(tst_a);
    tst_b = Qz.rotate(tst_b);

    OpenTissue::gjk::compute_closest_points(
      transformA
      , supportA
      , transformB
      , supportB
      , a
      , b
      , distance
      , iterations
      , status
      , absolute_tolerance
      , relative_tolerance
      , stagnation_tolerance
      , max_iterations
      , simplex_solver_policy
      );

    // Test if closest distance was correct
    BOOST_CHECK_CLOSE( 0.1, distance, 0.01 );

    // Test if status code was as expected
      BOOST_CHECK( status != OpenTissue::gjk::ABSOLUTE_CONVERGENCE );            // Would indicate penetration
      BOOST_CHECK( status != OpenTissue::gjk::INTERSECTION );                    // Would indicate penetration
      BOOST_CHECK( status != OpenTissue::gjk::ITERATING );                       // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::EXCEEDED_MAX_ITERATIONS_LIMIT );   // Would indicate internal error in GJK
      BOOST_CHECK( status != OpenTissue::gjk::NON_DESCEND_DIRECTION );           // Would indicate internal error in GJK
      //BOOST_CHECK( status != OpenTissue::gjk::RELATIVE_CONVERGENCE );          // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::SIMPLEX_EXPANSION_FAILED );      // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::STAGNATION );                    // Would indicate convergence to positive distance
      //BOOST_CHECK( status != OpenTissue::gjk::LOWER_ERROR_BOUND_CONVERGENCE ); // Would indicate convergence to positive distance

    // Test if closest points make sense
    BOOST_CHECK( fabs(a(0) - tst_a(0)) < 10e-6 );
    BOOST_CHECK( fabs(a(1) - tst_a(1)) < 10e-6 );
    BOOST_CHECK( fabs(a(2) - tst_a(2)) < 10e-6 );

    BOOST_CHECK( fabs(b(0) - tst_b(0)) < 10e-6 );
    BOOST_CHECK( fabs(b(1) - tst_b(1)) < 10e-6 );
    BOOST_CHECK( fabs(b(2) - tst_b(2)) < 10e-6 );
  }

}

BOOST_AUTO_TEST_SUITE_END();
