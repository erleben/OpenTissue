//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_generate_random.h>
#include <OpenTissue/core/math/big/big_is_orthonormal.h>
#include <OpenTissue/core/math/big/big_gram_schmidt.h>

#include <OpenTissue/core/math/big/big_generate_PD.h>
#include <OpenTissue/core/math/big/big_generate_PSD.h>
#include <OpenTissue/core/math/big/big_is_symmetric.h>


#include <OpenTissue/core/math/big/io/big_matlab_write.h>


#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_modified_gram_schmidt);

BOOST_AUTO_TEST_CASE(random_test_case)
{
  typedef ublas::compressed_matrix<double> matrix_type;

  matrix_type A;

  for(size_t tst=0;tst<5;++tst)
  {
    OpenTissue::math::big::generate_random(10, 10, A);

    bool not_ortho = !OpenTissue::math::big::is_orthonormal( A );
    BOOST_CHECK(not_ortho);

    OpenTissue::math::big::gram_schmidt(A);

    bool did_it = OpenTissue::math::big::is_orthonormal( A );
    BOOST_CHECK(did_it);
  }


  {
    using namespace OpenTissue::math::big;
    OpenTissue::math::big::fast_generate_PD( 10, A );

    std::cout << "A = " << A << ";" <<std::endl;

    bool is_ok = OpenTissue::math::big::is_symmetric( A );
    BOOST_CHECK(is_ok);

    OpenTissue::math::big::generate_PSD( 10, A, 0.5 );

    std::cout << "B = " << A << ";" <<std::endl;

    bool is_also_ok = OpenTissue::math::big::is_symmetric( A );
    BOOST_CHECK(is_also_ok);
  }
}

BOOST_AUTO_TEST_SUITE_END();
