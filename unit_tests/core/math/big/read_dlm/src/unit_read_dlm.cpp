//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/io/big_read_DLM.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_read_dlm);

BOOST_AUTO_TEST_CASE(my_test_case)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;

  std::string data_path = "dlm";

  matrix_type A;
  vector_type x;

  {
    std::string filename = "do_not_exist.wrong_type_extention";
    BOOST_CHECK_THROW( OpenTissue::math::big::read_dlm_matrix( filename, A), std::logic_error );
  }
  {
    std::string filename = "do_not_exist.wrong_type_extention";
    BOOST_CHECK_THROW( OpenTissue::math::big::read_dlm_vector( filename, x), std::logic_error );
  }
  double tolerance = 0.01;
  {
    std::string filename = data_path + "/4/A.dlm";
    bool success = OpenTissue::math::big::read_dlm_matrix( filename, A);
    BOOST_CHECK( success);
    BOOST_CHECK( A.size1() == 4 );
    BOOST_CHECK( A.size2() == 4 );
    BOOST_CHECK_CLOSE( double( A(0,0) ), 1.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(0,1) ), 2.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(0,2) ), 3.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(0,3) ), 4.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(1,0) ), 5.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(1,1) ), 6.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(1,2) ), 7.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(1,3) ), 8.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(2,0) ), 9.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(2,1) ), 10.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(2,2) ), 11.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(2,3) ), 12.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(3,0) ), 13.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(3,1) ), 14.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(3,2) ), 15.0, tolerance);
    BOOST_CHECK_CLOSE( double( A(3,3) ), 16.0, tolerance);
  }
  {
    std::string filename = data_path + "/4/b.dlm";
    bool success = OpenTissue::math::big::read_dlm_vector( filename, x);
    BOOST_CHECK( success);

    BOOST_CHECK( x.size() == 4 );
    BOOST_CHECK_CLOSE( double( x(0) ), 1.0, tolerance);
    BOOST_CHECK_CLOSE( double( x(1) ), 2.0, tolerance);
    BOOST_CHECK_CLOSE( double( x(2) ), 3.0, tolerance);
    BOOST_CHECK_CLOSE( double( x(3) ), 4.0, tolerance);
  }



}

BOOST_AUTO_TEST_SUITE_END();
