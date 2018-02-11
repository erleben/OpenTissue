//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_compute_index_sets.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


template <typename T>
class BoundFunction
{
public:

  typedef OpenTissue::math::ValueTraits<T> value_traits;

  bool m_is_lower;

public:

  BoundFunction(bool const & is_lower)
    : m_is_lower(is_lower)
  {}

  template<typename vector_type>
  T operator()(vector_type const & x, size_t const & i) const
  {
    return m_is_lower ?  -value_traits::one() : value_traits::one();
  }

};

BOOST_AUTO_TEST_SUITE(opentissue_math_big_compute_index_sets);

BOOST_AUTO_TEST_CASE(test_case)
{
  typedef double                    real_type;
  typedef ublas::vector<real_type>  vector_type;
  typedef ublas::vector<size_t>     idx_vector_type;

  vector_type x;
  vector_type y;

  x.resize(10,false);
  y.resize(10,false);

  BoundFunction<double> l(true);
  BoundFunction<double> u(false);

  x(0) =  0.0;   y(0) =  0.0;  // active
  x(1) = -1.0;   y(1) =  1.0;  // lower
  x(2) =  1.0;   y(2) = -1.0;  // upper
  x(3) = -2.0;   y(3) = -1.0;  // active
  x(4) =  2.0;   y(4) =  1.0;  // active
  x(5) =  3.0;   y(5) =  0.0;  // upper
  x(6) = -3.0;   y(6) =  0.0;  // lower
  x(7) =  0.5;   y(7) =  0.5;  // active
  x(8) = -0.5;   y(8) = -0.5;  // active
  x(9) =  0.25;   y(9) = 0.25; // active

  idx_vector_type bitmask;
  size_t cnt_active;
  size_t cnt_inactive;
  OpenTissue::math::optimization::compute_index_sets( y, x, l, u, bitmask, cnt_active, cnt_inactive );

  BOOST_CHECK( cnt_active == 6 );
  BOOST_CHECK( cnt_inactive == 4 );

  BOOST_CHECK( bitmask(0) == OpenTissue::math::optimization::IN_ACTIVE );
  BOOST_CHECK( bitmask(1) == OpenTissue::math::optimization::IN_LOWER );
  BOOST_CHECK( bitmask(2) == OpenTissue::math::optimization::IN_UPPER );
  BOOST_CHECK( bitmask(3) == OpenTissue::math::optimization::IN_ACTIVE );
  BOOST_CHECK( bitmask(4) == OpenTissue::math::optimization::IN_ACTIVE );
  BOOST_CHECK( bitmask(5) == OpenTissue::math::optimization::IN_UPPER );
  BOOST_CHECK( bitmask(6) == OpenTissue::math::optimization::IN_LOWER );
  BOOST_CHECK( bitmask(7) == OpenTissue::math::optimization::IN_ACTIVE );
  BOOST_CHECK( bitmask(8) == OpenTissue::math::optimization::IN_ACTIVE );
  BOOST_CHECK( bitmask(9) == OpenTissue::math::optimization::IN_ACTIVE );
}

BOOST_AUTO_TEST_SUITE_END();
