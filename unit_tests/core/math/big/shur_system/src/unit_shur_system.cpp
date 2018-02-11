//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_gmres.h>
#include <OpenTissue/core/math/big/big_shur_system.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_shur_system);

BOOST_AUTO_TEST_CASE(test_case)
{

  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef vector_type::size_type           size_type;
  typedef double                           real_type;

  typedef OpenTissue::math::big::GMRESFunctor solve_functor;

  {
    solve_functor solver;

    matrix_type A_aa;
    A_aa.resize(4,4,false);
    matrix_type A_ab;
    A_ab.resize(4,6,false);
    matrix_type C;
    C.resize(6,4,false);
    matrix_type invD;
    invD.resize(6,6,false);
    A_aa(0,0) = 4.017000e+000;	  A_aa(0,1) = 3.330600e+000;	  A_aa(0,2) = 1.327000e+000;	  A_aa(0,3) = 2.505800e+000;	
    A_aa(1,0) = 3.330600e+000;	  A_aa(1,1) = 5.279000e+000;	  A_aa(1,2) = 2.207500e+000;	  A_aa(1,3) = 3.907900e+000;	
    A_aa(2,0) = 1.327000e+000;	  A_aa(2,1) = 2.207500e+000;	  A_aa(2,2) = 1.367300e+000;	  A_aa(2,3) = 1.459200e+000;	
    A_aa(3,0) = 2.505800e+000;	  A_aa(3,1) = 3.907900e+000;	  A_aa(3,2) = 1.459200e+000;	  A_aa(3,3) = 3.501400e+000;	

    A_ab(0,0) = 3.189600e+000;	  A_ab(0,1) = 2.608600e+000;	  A_ab(0,2) = 3.403200e+000;	  A_ab(0,3) = 3.517500e+000;	  A_ab(0,4) = 3.797800e+000;	  A_ab(0,5) = 2.173200e+000;	
    A_ab(1,0) = 3.141500e+000;	  A_ab(1,1) = 3.671200e+000;	  A_ab(1,2) = 3.866500e+000;	  A_ab(1,3) = 3.994500e+000;	  A_ab(1,4) = 4.309800e+000;	  A_ab(1,5) = 2.991400e+000;	
    A_ab(2,0) = 1.351200e+000;	  A_ab(2,1) = 1.610800e+000;	  A_ab(2,2) = 1.710100e+000;	  A_ab(2,3) = 1.722400e+000;	  A_ab(2,4) = 1.631200e+000;	  A_ab(2,5) = 1.470500e+000;	
    A_ab(3,0) = 2.297200e+000;	  A_ab(3,1) = 2.858200e+000;	  A_ab(3,2) = 3.173400e+000;	  A_ab(3,3) = 2.865900e+000;	  A_ab(3,4) = 3.305500e+000;	  A_ab(3,5) = 2.174900e+000;	

    C(0,0) = 3.189600e+000;	  C(0,1) = 3.141500e+000;	  C(0,2) = 1.351200e+000;	  C(0,3) = 2.297200e+000;	
    C(1,0) = 2.608600e+000;	  C(1,1) = 3.671200e+000;	  C(1,2) = 1.610800e+000;	  C(1,3) = 2.858200e+000;	
    C(2,0) = 3.403200e+000;	  C(2,1) = 3.866500e+000;	  C(2,2) = 1.710100e+000;	  C(2,3) = 3.173400e+000;	
    C(3,0) = 3.517500e+000;	  C(3,1) = 3.994500e+000;	  C(3,2) = 1.722400e+000;	  C(3,3) = 2.865900e+000;	
    C(4,0) = 3.797800e+000;	  C(4,1) = 4.309800e+000;	  C(4,2) = 1.631200e+000;	  C(4,3) = 3.305500e+000;	
    C(5,0) = 2.173200e+000;	  C(5,1) = 2.991400e+000;	  C(5,2) = 1.470500e+000;	  C(5,3) = 2.174900e+000;	

    invD(0,0) = 6.146263e+000;	  invD(0,1) = 7.993048e-001;	  invD(0,2) = -3.021859e+000;	  invD(0,3) = -9.747487e+000;	  invD(0,4) = 4.740940e+000;	  invD(0,5) = 2.547374e+000;	
    invD(1,0) = 7.993048e-001;	  invD(1,1) = 1.690192e+000;	  invD(1,2) = -2.131304e-001;	  invD(1,3) = -1.549999e+000;	  invD(1,4) = -3.341985e-001;	  invD(1,5) = 7.856728e-002;	
    invD(2,0) = -3.021859e+000;	  invD(2,1) = -2.131304e-001;	  invD(2,2) = 3.100454e+000;	  invD(2,3) = 4.289924e+000;	  invD(2,4) = -3.146358e+000;	  invD(2,5) = -1.618643e+000;	
    invD(3,0) = -9.747487e+000;	  invD(3,1) = -1.549999e+000;	  invD(3,2) = 4.289924e+000;	  invD(3,3) = 1.868662e+001;	  invD(3,4) = -8.817597e+000;	  invD(3,5) = -5.281455e+000;	
    invD(4,0) = 4.740940e+000;	  invD(4,1) = -3.341985e-001;	  invD(4,2) = -3.146358e+000;	  invD(4,3) = -8.817597e+000;	  invD(4,4) = 5.824081e+000;	  invD(4,5) = 2.703271e+000;	
    invD(5,0) = 2.547374e+000;	  invD(5,1) = 7.856728e-002;	  invD(5,2) = -1.618643e+000;	  invD(5,3) = -5.281455e+000;	  invD(5,4) = 2.703271e+000;	  invD(5,5) = 2.475215e+000;	

    vector_type rhs_a;
    rhs_a.resize(4,false);
    rhs_a(0) = 1.529457e+001;	  rhs_a(1) = 1.828775e+001;	  rhs_a(2) = 7.956576e+000;	  rhs_a(3) = 1.384063e+001;	
    vector_type rhs_b;
    rhs_b.resize(6,false);
    rhs_b(0) = 1.306312e+001;	  rhs_b(1) = 1.374039e+001;	  rhs_b(2) = 1.582071e+001;	  rhs_b(3) = 1.597036e+001;	  rhs_b(4) = 1.743755e+001;	  rhs_b(5) = 1.169438e+001;	
    vector_type dx_a;
    vector_type dx_b;
    OpenTissue::math::big::shur_system( A_aa, A_ab, C, invD, rhs_a, rhs_b, dx_a, dx_b, solver );

    real_type tol  = real_type(5.0);

    BOOST_CHECK_CLOSE( real_type( dx_a(0) ), real_type(5.828000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_a(1) ), real_type(4.235000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_a(2) ), real_type(5.155000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_a(3) ), real_type(3.340000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_b(0) ), real_type(4.329000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_b(1) ), real_type(2.259000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_b(2) ), real_type(5.798000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_b(3) ), real_type(7.604000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_b(4) ), real_type(5.298000e-001), tol );
    BOOST_CHECK_CLOSE( real_type( dx_b(5) ), real_type(6.405000e-001), tol );
  }

}

BOOST_AUTO_TEST_SUITE_END();
