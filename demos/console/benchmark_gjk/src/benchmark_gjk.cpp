//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk.h>
#include <OpenTissue/core/geometry/geometry_obb.h>
#include <OpenTissue/utility/utility_timer.h>


/**
@file   This file contains a benchmark test comparing our old GJK implementation with our new GJK implementation.
*/


void old_implementation()
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::coordsys_type                        coordsys_type;
  typedef math_types::real_type                            real_type;

  vector3_type p_a;                   
  vector3_type p_b;
  OpenTissue::gjk::obsolete::detail::GJK<vector3_type > gjk;          
  OpenTissue::geometry::OBB<math_types> A;
  OpenTissue::geometry::OBB<math_types> B;
  A.init(2.0,2.0,2.0);
  B.init(2.0,2.0,2.0);
  coordsys_type  Awcs;
  coordsys_type  Bwcs;
  Awcs.Q().identity();
  Bwcs.Q().identity();
  OpenTissue::utility::Timer<double> duration;
  duration.start();
  for(int i=0;i<10000;++i)
  {
    OpenTissue::math::random( Awcs.T() );
    OpenTissue::math::random( Bwcs.T() );
    A.place(Awcs);
    B.place(Bwcs);
    gjk.get_closest_points(A,B,p_a,p_b);
  }
  duration.stop();
  std::cout << "old impl 10000 random test runs: " << duration() << " seconds" << std::endl;
}

void new_implementation()
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;

  typedef math_types::quaternion_type                      quaternion_type;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef math_types::coordsys_type                        transformation_type;
  typedef math_types::value_traits                         value_traits;

  OpenTissue::gjk::VoronoiSimplexSolverPolicy const simplex_solver_policy = OpenTissue::gjk::VoronoiSimplexSolverPolicy();

  OpenTissue::gjk::Box<math_types> supportA;
  OpenTissue::gjk::Box<math_types> supportB;

  supportA.half_extent() = vector3_type(1.0,1.0,1.0);
  supportB.half_extent() = vector3_type(1.0,1.0,1.0);

  size_t    const max_iterations       = 100u;
  real_type const absolute_tolerance   = boost::numeric_cast<real_type>(10e-6);
  real_type const relative_tolerance   = boost::numeric_cast<real_type>(10e-6);
  real_type const stagnation_tolerance = boost::numeric_cast<real_type>(10e-15);

  transformation_type transformA;
  transformation_type transformB;

  vector3_type a;
  vector3_type b;
  size_t iterations     = 0u;
  size_t status         = 0u;
  real_type distance    = value_traits::infinity();

  transformA.Q().identity();
  transformB.Q().identity();

  OpenTissue::utility::Timer<double> duration;
  duration.start();
  for(int i=0;i<10000;++i)
  {

    OpenTissue::math::random( transformA.T() );
    OpenTissue::math::random( transformB.T() );

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
  }
  duration.stop();
  std::cout << "new impl 10000 random test runs: " << duration() << " seconds" << std::endl;
}


int main( int argc, char **argv )
{
  old_implementation();
  new_implementation();

  return 0;
}
