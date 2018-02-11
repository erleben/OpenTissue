//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_compute_generalized_minimal_map.h>
#include <OpenTissue/core/math/optimization/optimization_compute_natural_merit.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


template <typename T>
class BoundFunction
{
public:

  T    m_value;

public:

  BoundFunction(T const & value)
    : m_value(value)
  {}

  template<typename vector_type>
  T operator()(vector_type const & x, size_t const & i) const  {    return m_value;  }

};

BOOST_AUTO_TEST_SUITE(opentissue_math_big_compute_generalized_minimal_map);

BOOST_AUTO_TEST_CASE(test_case)
{


  ublas::vector<double> x;
  ublas::vector<double> y;
  ublas::vector<double> H;

  x.resize(10,false);
  y.resize(10,false);
  H.resize(10,false);

  OpenTissue::math::Random<double> value(0.0,1.0);

  x(0) = value();
  x(1) = value();
  x(2) = value();
  x(3) = value();
  x(4) = value();
  x(5) = value();
  x(6) = value();
  x(7) = value();
  x(8) = value();
  x(9) = value();

  y(0) = value();
  y(1) = value();
  y(2) = value();
  y(3) = value();
  y(4) = value();
  y(5) = value();
  y(6) = value();
  y(7) = value();
  y(8) = value();
  y(9) = value();

  BoundFunction<double> l(-value());
  BoundFunction<double> u( value());

  OpenTissue::math::optimization::compute_generalized_minimal_map(y,l,u,x, H );

  double theta = OpenTissue::math::optimization::compute_natural_merit( H );
  BOOST_CHECK ( theta >= 0.0 );

}

BOOST_AUTO_TEST_SUITE_END();
