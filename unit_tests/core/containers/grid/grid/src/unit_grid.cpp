//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/containers/grid/grid.h>
#include <cmath> 

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


template <typename grid_type>
void grid_test( grid_type & G )
{
  typedef typename grid_type::value_type     value_type;
  typedef typename grid_type::math_types     math_types;
  typedef typename math_types::vector3_type  vector3_type;
  typedef typename math_types::real_type     real_type;

  value_type tol = value_type(0.01);

  size_t I = 3;
  size_t J = 3;
  size_t K = 3;

  // testing creation
  {
    vector3_type min_coord (-1.0, -1.0, -1.0 );
    vector3_type max_coord ( 1.0,  1.0,  1.0 );

    G.create( min_coord, max_coord, I, J, K);

    BOOST_CHECK( I == G.I() );
    BOOST_CHECK( J == G.J() );
    BOOST_CHECK( K == G.K() );
    BOOST_CHECK( (I*J*K) == G.size() );
  }

  grid_type const & H = G;

  // testing read and write access to grid
  {
    size_t linear_index = 0;
    value_type input    = 0;
    for(size_t k = 0;k<K;++k)
    {
      for(size_t j = 0;j<J;++j)
      {
        for(size_t i = 0;i<I;++i)
        {
          value_type output0 = G(i,j,k);
          BOOST_CHECK_CLOSE(output0, G.unused(), tol);

          G(i,j,k) = input;

          value_type const output1 = H(i,j,k);
          BOOST_CHECK_CLOSE(input, output1, tol);

          value_type const output2 = H.get_value(i,j,k);
          BOOST_CHECK_CLOSE(input, output2, tol);

          G( linear_index ) = input;

          value_type const output3 = H(linear_index);
          BOOST_CHECK_CLOSE(input, output3, tol);

          value_type const output4 = H.get_value(linear_index);
          BOOST_CHECK_CLOSE(input, output4, tol);

          linear_index += 1;
          input = input + 1;
        }
      }
    }
  }


  // iterator testing
  {
    grid_type cpy = G;

    typedef typename grid_type::iterator iterator;
    typedef typename grid_type::const_iterator const_iterator;

    iterator c = cpy.begin();
    iterator c_end = cpy.end();
    const_iterator g = H.begin();
    const_iterator g_end = H.end();

    for(;c!=c_end;++c,++g)
    {
      BOOST_CHECK_CLOSE( *c, *g, tol);
    }
  }


  // index iterator testing
  {
    grid_type cpy = G;

    typedef typename grid_type::index_iterator         index_iterator;
    typedef typename grid_type::const_index_iterator   const_index_iterator;

    index_iterator c = cpy.begin();
    index_iterator c_end = cpy.end();
    const_index_iterator g = H.begin();
    const_index_iterator g_end = H.end();

    for(size_t k = 0;k<K;++k)
    {
      for(size_t j = 0;j<J;++j)
      {
        for(size_t i = 0;i<I;++i)
        {
          BOOST_CHECK_CLOSE( *c, *g, tol);

          BOOST_CHECK( c.i() == i );
          BOOST_CHECK( c.j() == j );
          BOOST_CHECK( c.k() == k );

          BOOST_CHECK( g.i() == i );
          BOOST_CHECK( g.j() == j );
          BOOST_CHECK( g.k() == k );

          ++c;
          ++g;

        }
      }
    }
  }


  // min_element and max_element testing
  {
    value_type const min_value = - G(2,2,2)*2;
    value_type const max_value = - min_value;
    G(1,1,1) = min_value;
    G(0,1,1) = max_value;
    value_type tst_min = OpenTissue::grid::min_element( G );
    value_type tst_max = OpenTissue::grid::max_element( G );
    BOOST_CHECK_CLOSE( tst_min, min_value, tol);
    BOOST_CHECK_CLOSE( tst_max, max_value, tol);
  }


  // fabs testing
  {
    G = OpenTissue::grid::fabs(G);
  }

  // negate testing
  {
    G = OpenTissue::grid::negate(G);
  }
  // scale testing
  {
    value_type value = value_type(2);
    G = OpenTissue::grid::scale(G, value );
  }


}

BOOST_AUTO_TEST_SUITE(opentissue_grid);

BOOST_AUTO_TEST_CASE(test_cases)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  {
    typedef OpenTissue::grid::Grid<float,math_types>         grid_type;
    grid_type G;
    grid_test(G);
  }
  {
    typedef OpenTissue::grid::Grid<double,math_types>        grid_type;
    grid_type G;
    grid_test(G);
  }
}

BOOST_AUTO_TEST_SUITE_END();
