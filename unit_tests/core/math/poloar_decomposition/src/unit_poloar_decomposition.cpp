//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>
#include <OpenTissue/core/math/math_polar_decomposition.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>
#include <iostream>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_math_polar_decomposition);

  BOOST_AUTO_TEST_CASE(eigen_method)
  {
    typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
    typedef math_types::vector3_type                         vector3_type;
    typedef math_types::matrix3x3_type                       matrix3x3_type;
    typedef math_types::real_type                            real_type;
    typedef math_types::index_type                           index_type;
    typedef math_types::value_traits                         value_traits;

    real_type epsilon = 10e-7;
    matrix3x3_type A,R,S,D;

    for(index_type i=0;i<100000;++i)
    {
      OpenTissue::math::random(S);
      S = OpenTissue::math::trans(S)*S;
      OpenTissue::math::random(A);
      R = OpenTissue::math::ortonormalize( A );
      A = R*S;
      R = OpenTissue::math::diag(1.0);
      S = OpenTissue::math::diag(1.0);
      bool success = OpenTissue::math::polar_decomposition::eigen(A,R,S);
      if(success)
      {
        bool right_handed = det(R) > value_traits::zero();
        BOOST_CHECK( right_handed );
        
        D = A - R*S;
        real_type maximum_deviation =  max_value(  fabs(D) );

        //BOOST_CHECK_CLOSE( maximum_deviation, value_traits::zero(), tol);
        BOOST_CHECK( maximum_deviation<epsilon );
      }
    }
  }

BOOST_AUTO_TEST_SUITE_END();
