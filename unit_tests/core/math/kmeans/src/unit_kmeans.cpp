//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/math/math_kmeans.h>

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

BOOST_AUTO_TEST_SUITE(opentissue_math_kmeans);

BOOST_AUTO_TEST_CASE(simple_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::matrix3x3_type                       matrix3x3_type;
  typedef math_types::real_type                            real_type;
  typedef math_types::index_type                           index_type;
  typedef math_types::value_traits                         value_traits;
  typedef std::vector<vector3_type>                        vector_container;
  typedef std::vector<size_t>                              index_container;

  vector_container features;
  size_t index = 0;
  features.resize(40);
  real_type lower = - value_traits::half();
  real_type upper = value_traits::half();

  vector3_type center[4];
  center[0] = vector3_type( value_traits::half(), value_traits::half(), -value_traits::two() );
  center[1] = vector3_type( -value_traits::four(), value_traits::zero(), -value_traits::half() );
  center[2] = vector3_type( value_traits::half(), value_traits::four(), value_traits::half() );
  center[3] = vector3_type( value_traits::half(), -value_traits::two(), value_traits::four() );

  for(size_t i = 0;i<10;++i,++index)
  {
    OpenTissue::math::random( features[index], lower, upper );
    features[index] += center[0];
  }
  for(size_t i = 0;i<10;++i,++index)
  {
    OpenTissue::math::random( features[index], lower, upper );
    features[index] += center[1];
  }
  for(size_t i = 0;i<10;++i,++index)
  {
    OpenTissue::math::random( features[index], lower, upper );
    features[index] += center[2];
  }
  for(size_t i = 0;i<10;++i,++index)
  {
    OpenTissue::math::random( features[index], lower, upper );
    features[index] += center[3];
  }

  vector_container cluster_centers;
  index_container cluster_indexes;

  size_t K = 4;
  size_t iteration = 0u;
  size_t max_iterations = 50u;

  // K-means may get caught by local minimas, in such cases clusters
  // seem to ``melt'' together unexpected.
  //
  // If k-means gets caught by a local minima then all the following
  // unit-tests is likely to fail!
  //
  OpenTissue::math::kmeans( 
    features.begin()
    , features.end()
    , cluster_centers
    , cluster_indexes
    , K
    , iteration
    , max_iterations
    );

  BOOST_CHECK( iteration < max_iterations );

  for(size_t i = 0;i<40;++i)
    std::cout << cluster_indexes[i] << " ";
  std::cout << std::endl;

  size_t cluster_order[4];
  cluster_order[0] = cluster_indexes[0];
  cluster_order[1] = cluster_indexes[10];
  cluster_order[2] = cluster_indexes[20];
  cluster_order[3] = cluster_indexes[30];

  BOOST_CHECK( cluster_order[0] != cluster_order[1] );
  BOOST_CHECK( cluster_order[0] != cluster_order[2] );
  BOOST_CHECK( cluster_order[0] != cluster_order[3] );
  BOOST_CHECK( cluster_order[1] != cluster_order[2] );
  BOOST_CHECK( cluster_order[1] != cluster_order[3] );
  BOOST_CHECK( cluster_order[2] != cluster_order[3] );

  for(size_t i = 0;i<10;++i)
    BOOST_CHECK( cluster_indexes[i] ==  cluster_order[0] );
  for(size_t i = 10;i<20;++i)
    BOOST_CHECK( cluster_indexes[i] ==  cluster_order[1] );
  for(size_t i = 20;i<30;++i)
    BOOST_CHECK( cluster_indexes[i] ==  cluster_order[2] );
  for(size_t i = 30;i<40;++i)
    BOOST_CHECK( cluster_indexes[i] ==  cluster_order[3] );

  for(size_t c = 0;c<K;++c)
  {
    std::cout << cluster_centers[ cluster_order[c] ] << std::endl;
    real_type dist = OpenTissue::math::length( center[c] - cluster_centers[ cluster_order[c] ] );
    BOOST_CHECK(dist < value_traits::half() );
  }
}

BOOST_AUTO_TEST_SUITE_END();
