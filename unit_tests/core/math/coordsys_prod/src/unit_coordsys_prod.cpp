//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>


#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_coordsys_prod);

BOOST_AUTO_TEST_CASE(simple_test)
{
  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;

  typedef math_types::value_traits     value_traits;
  typedef math_types::real_type        T;
  typedef math_types::vector3_type     V;
  typedef math_types::quaternion_type  Q;
  typedef math_types::coordsys_type    X;

  {
    X I;
    I.identity();

    X L;
    L.T() = V(1.0,2.0,3.0);
    L.Q().Ru( value_traits::pi(), V(1.0, 2.0, 3.0) );

    X R = OpenTissue::math::prod( L, I );

    BOOST_CHECK( fabs( R.T()(0) - L.T()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(1) - L.T()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(2) - L.T()(2) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().s() - L.Q().s() ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(0) - L.Q().v()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(1) - L.Q().v()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(2) - L.Q().v()(2) ) < 10e-10 );
  }
  {
    X I;
    I.identity();

    X L;
    L.T() = V(1.0,2.0,3.0);
    L.Q().Ru( value_traits::pi(), V(1.0, 2.0, 3.0) );

    X R = OpenTissue::math::prod( I, L );

    BOOST_CHECK( fabs( R.T()(0) - L.T()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(1) - L.T()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(2) - L.T()(2) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().s() - L.Q().s() ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(0) - L.Q().v()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(1) - L.Q().v()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(2) - L.Q().v()(2) ) < 10e-10 );
  }
  {
    X I;
    I.identity();

    X R = OpenTissue::math::prod( I, I );

    BOOST_CHECK( fabs( R.T()(0) - I.T()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(1) - I.T()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(2) - I.T()(2) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().s() - I.Q().s() ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(0) - I.Q().v()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(1) - I.Q().v()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(2) - I.Q().v()(2) ) < 10e-10 );
  }
  {
    X I;
    I.identity();

    X L;
    L.T() = V(1.0,2.0,3.0);
    L.Q().Ru( value_traits::pi(), V(1.0, 2.0, 3.0) );

    X invL = OpenTissue::math::inverse( L );
    X R = OpenTissue::math::prod( invL, L );

    BOOST_CHECK( fabs( R.T()(0) - I.T()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(1) - I.T()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.T()(2) - I.T()(2) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().s() - I.Q().s() ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(0) - I.Q().v()(0) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(1) - I.Q().v()(1) ) < 10e-10 );
    BOOST_CHECK( fabs( R.Q().v()(2) - I.Q().v()(2) ) < 10e-10 );
  }
}

BOOST_AUTO_TEST_SUITE_END();
