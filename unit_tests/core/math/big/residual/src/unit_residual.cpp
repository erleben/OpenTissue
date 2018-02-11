//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_residual.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_residual);

BOOST_AUTO_TEST_CASE(random_test_case)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;

  matrix_type A;
  vector_type x;
  vector_type b;
  vector_type r;
  vector_type tst;

  A.resize(10,10,false);
  x.resize(10,false);
  b.resize(10,false);
  r.resize(10,false);
  tst.resize(10,false);

  OpenTissue::math::Random<double> value(0.0,1.0);

  for(size_t iterations=0;iterations < 100;++iterations)
  {
    for(size_t i=0;i<A.size1();++i)
    {
      x(i) = value();
      b(i) = value();
      r(i) = value();
      for(size_t j=0;j<A.size2();++j)
        A(i,j) = value();
    }
    ublas::noalias(tst) = b - ublas::prod(A,x);
    OpenTissue::math::big::residual( A,x,b,r);
    double tol = 0.01;
    for(size_t i=0;i<A.size1();++i)
      BOOST_CHECK_CLOSE( double( r(i) ), double( tst(i) ), tol );
  }
}

BOOST_AUTO_TEST_SUITE_END();
