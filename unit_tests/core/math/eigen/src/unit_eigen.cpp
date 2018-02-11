//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

template<typename vector3_type,typename matrix3x3_type>
void eigen_value_decomposition_test(vector3_type d,matrix3x3_type R)
{
  using std::fabs;

  typedef typename vector3_type::value_type    real_type;
  typedef typename vector3_type::value_traits  value_traits;

  real_type tol = 0.0001;

  vector3_type r1 = vector3_type(R(0,0),R(1,0),R(2,0));
  vector3_type r2 = vector3_type(R(0,1),R(1,1),R(2,1));
  vector3_type r3 = vector3_type(R(0,2),R(1,2),R(2,2));
  real_type    d0 = d(0);
  real_type    d1 = d(1);
  real_type    d2 = d(2);

  matrix3x3_type D = diag(d);
  matrix3x3_type A = R*D*trans(R);
  BOOST_CHECK( is_symmetric(A,tol) );

  matrix3x3_type V;
  OpenTissue::math::eigen(A, V, d);

  vector3_type e1 = vector3_type(V(0,0),V(1,0),V(2,0));
  vector3_type e2 = vector3_type(V(0,1),V(1,1),V(2,1));
  vector3_type e3 = vector3_type(V(0,2),V(1,2),V(2,2));

  real_type determinant = det(V);
  BOOST_CHECK_CLOSE( fabs( determinant ), value_traits::one(), tol );

  real_type epsilon = 10e-7;

  matrix3x3_type Itest = trans(V)*V;

  BOOST_CHECK_CLOSE( Itest(0,0), value_traits::one() , tol );
  BOOST_CHECK_CLOSE( Itest(1,1), value_traits::one() , tol );
  BOOST_CHECK_CLOSE( Itest(2,2), value_traits::one() , tol );
  //--- The check close versin behaves strange when rhs is exactly
  //--- zero?, so I use the check-version instead...
  //BOOST_CHECK_CLOSE(      Itest(0,1) , value_traits::zero(), tol );
  //BOOST_CHECK_CLOSE(      Itest(0,2) , value_traits::zero(), tol );
  //BOOST_CHECK_CLOSE(      Itest(1,0) , value_traits::zero(), tol );
  //BOOST_CHECK_CLOSE(      Itest(1,2) , value_traits::zero(), tol );
  //BOOST_CHECK_CLOSE(      Itest(2,0) , value_traits::zero(), tol );
  //BOOST_CHECK_CLOSE(      Itest(2,1) , value_traits::zero(), tol );
  BOOST_CHECK( fabs(Itest(0,1))<epsilon );
  BOOST_CHECK( fabs(Itest(0,2))<epsilon );
  BOOST_CHECK( fabs(Itest(1,0))<epsilon );
  BOOST_CHECK( fabs(Itest(1,2))<epsilon );
  BOOST_CHECK( fabs(Itest(2,0))<epsilon );
  BOOST_CHECK( fabs(Itest(2,1))<epsilon );

  matrix3x3_type Atest = A - V*diag(d)*trans(V);

  BOOST_CHECK( fabs(Atest(0,0))<epsilon );
  BOOST_CHECK( fabs(Atest(0,1))<epsilon );
  BOOST_CHECK( fabs(Atest(0,2))<epsilon );
  BOOST_CHECK( fabs(Atest(1,0))<epsilon );
  BOOST_CHECK( fabs(Atest(1,1))<epsilon );
  BOOST_CHECK( fabs(Atest(1,2))<epsilon );
  BOOST_CHECK( fabs(Atest(2,0))<epsilon );
  BOOST_CHECK( fabs(Atest(2,1))<epsilon );
  BOOST_CHECK( fabs(Atest(2,2))<epsilon );

  bool match1 = fabs( d0 - d(0) ) < epsilon &&  
                fabs( d1 - d(1) ) < epsilon &&
                fabs( d2 - d(2) ) < epsilon ;

  bool match2 = fabs( d0 - d(0) )< epsilon &&
                fabs( d1 - d(2) )< epsilon &&
                fabs( d2 - d(1) )< epsilon ;

  bool match3 = fabs( d0 - d(1) )< epsilon &&
                fabs( d1 - d(0) )< epsilon &&
                fabs( d2 - d(2) )< epsilon ;

  bool match4 = fabs( d0 - d(1) )< epsilon &&
                fabs( d1 - d(2) )< epsilon &&
                fabs( d2 - d(0) )< epsilon ;

  bool match5 = fabs( d0 - d(2) )< epsilon &&
                fabs( d1 - d(0) )< epsilon &&
                fabs( d2 - d(1) )< epsilon ;

  bool match6 = fabs( d0 - d(2) )< epsilon &&
                fabs( d1 - d(1) )< epsilon &&
                fabs( d2 - d(0) )< epsilon ;

  BOOST_CHECK( match1 || match2 || match3 || match4 || match5 || match6 );
}

BOOST_AUTO_TEST_SUITE(opentissue_math_eigen);

  BOOST_AUTO_TEST_CASE(random_testing)
  {
    typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
    typedef math_types::vector3_type                         vector3_type;
    typedef math_types::matrix3x3_type                       matrix3x3_type;
    typedef math_types::real_type                            real_type;

    matrix3x3_type R;
    vector3_type d;

    for(int i= 0;i<100;++i)
    {
      //--- non-negative eigen-values
      random(d,0,1);
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- non-positive eigen-values
      random(d,-1,0);
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- one zero eigen-values
      random(d,0,1);
      d(0) = 0;
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- one zero eigen-values
      random(d,0,1);
      d(1) = 0;
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- one zero eigen-values
      random(d,0,1);
      d(2) = 0;
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- two zero eigen-values
      random(d,0,1);
      d(0) = 0;
      d(1) = 0;
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- two zero eigen-values
      random(d,0,1);
      d(0) = 0;
      d(2) = 0;
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- two zero eigen-values
      random(d,0,1);
      d(1) = 0;
      d(2) = 0;
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- three zero eigen-values
      d.clear();
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- multiplicity of 3
      random(d,0,1);
      d(1) = d(0);
      d(2) = d(0);
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- multiplicity of 2
      random(d,0,1);
      d(1) = d(0);
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- multiplicity of 2
      random(d,0,1);
      d(2) = d(0);
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);

      //--- multiplicity of 2
      random(d,0,1);
      d(2) = d(1);
      random(R);
      R = ortonormalize(R);
      eigen_value_decomposition_test(d,R);
    }
  }

BOOST_AUTO_TEST_SUITE_END();
