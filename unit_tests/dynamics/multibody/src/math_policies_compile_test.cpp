//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/math/mbd_default_math_policy.h>
#include <OpenTissue/dynamics/mbd/math/mbd_optimized_ublas_math_policy.h>

#include <iostream>

template<typename math_policy>
void math_policy_compile_test()
{
  using namespace OpenTissue::math::big;

  typedef typename math_policy::real_type           real_type;
  typedef typename math_policy::size_type           size_type;
  typedef typename math_policy::value_traits        value_traits;
  typedef typename math_policy::vector3_type        vector3_type;
  typedef typename math_policy::matrix3x3_type      matrix3x3_type;
  typedef typename math_policy::quaternion_type     quaternion_type;
  typedef typename math_policy::coordsys_type       coordsys_type;
  typedef typename math_policy::idx_vector_type     idx_vector_type;
  typedef typename math_policy::vector_type         vector_type;
  typedef typename math_policy::matrix_type         matrix_type;
  typedef typename math_policy::system_matrix_type  system_matrix_type;
  typedef typename math_policy::idx_vector_range    idx_vector_range;
  typedef typename math_policy::vector_range        vector_range;
  typedef typename math_policy::matrix_range        matrix_range;

  {
    real_type s0;
    s0 = real_type();
    real_type s1 = real_type();
    real_type s2 = s1;
    real_type s3(s2);
    real_type s4(value_traits::zero());
    real_type s5 = value_traits::zero();
    real_type s6(value_traits::one());
    real_type s7 = value_traits::one();
    real_type s8(value_traits::two());
    real_type s9 = value_traits::two();
    real_type s10(value_traits::pi());
    real_type s11 = value_traits::pi();
    real_type s12(value_traits::infinity());
    real_type s13 = value_traits::infinity();
    
    s13 = s1*s2*s3*s4*s5*s6*s7*s8*s9*s10*s11*s12; // To get rid of compiler warning about unused variables
  }
  {
    size_type start1 = 0;
    size_type stop1 = 0;

    idx_vector_type iv;
    math_policy::subrange( iv, start1, stop1 );

    size_type i = 0;
    math_policy::subrange( iv, start1, stop1 )(i);
  }
  {
    size_type start1 = 0;
    size_type stop1 = 0;
    vector_type v;
    math_policy::subrange( v, start1, stop1 );

    size_type i = 0;
    math_policy::subrange( v, start1, stop1 )(i);
  }
  {
    size_type start1 = 0;
    size_type stop1 = 0;
    size_type start2 = 0;
    size_type stop2 = 0;

    matrix_type M;
    math_policy::subrange( M, start1, stop1, start2, stop2 );

    size_type i = 0;
    size_type j = 0;
    math_policy::subrange( M, start1, stop1, start2, stop2 )(i,j);
  }
  {
    matrix_type A;

    size_type m = 0;
    size_type n = 0;
    math_policy::resize(A,m,n);
  }
  {
    idx_vector_type iv;
    vector_type v;

    size_type m = 0;
    math_policy::resize(v,m);
    math_policy::resize(iv,m);
  }
  {
    vector_type x;
    size_type n = 0;
    math_policy::get_dimension(x,n);
  }
  {
    matrix_type A;

    size_type m = 0;
    size_type n = 0;
    math_policy::get_dimensions(A,m,n);
  }
  {
    matrix_type A;
    vector_type x;
    vector_type y;
    math_policy::prod_add(A,x,y);
  }
  {
    matrix_type A;
    vector_type x;
    vector_type y;
    real_type s;
    math_policy::prod_add(A,x,y,s);
  }
  {
    matrix_type A;
    vector_type x;
    vector_type y;
    vector_type b;
    math_policy::prod_minus(A,x,b,y);
  }
  {
    matrix_type A;
    vector_type x;
    vector_type y;
    math_policy::prod_trans(A,x,y);
  }
  {
    matrix_type A;
    vector_type x;
    vector_type y;
    vector_type b;
    math_policy::prod_trans(A,x,b,y);
  }
  {
    vector_type x;
    real_type s;
    math_policy::prod(x,s);
  }
  {
    vector_type x;
    vector_type y;
    math_policy::assign_minus(x,y);
  }
  {
    matrix_type A;
    std::cout << A << std::endl;
  }
  {
    vector_type a;
    std::cout << a << std::endl;
  }
  {
    matrix3x3_type A;
    std::cout << A << std::endl;
  }
  {
    vector3_type a;
    std::cout << a << std::endl;
  }
  {
    quaternion_type q;
    std::cout << q << std::endl;
  }
  {
    coordsys_type c;
    std::cout << c << std::endl;
  }
  {
    system_matrix_type A;
    vector_type x;
    size_type i = 0;
    math_policy::row_prod(A,i,x);
  }
  {
    system_matrix_type A;
    vector_type x;
    vector_type b;
    vector_type y;
    math_policy::prod(A,x,b,y);
  }  
  {
    system_matrix_type A;
    real_type dx;
    size_type i = 0;
    math_policy::update_system_matrix(A,i,dx);
  }
  {
    system_matrix_type A;
    vector_type x;
    math_policy::init_system_matrix(A,x);
  }
  {
    system_matrix_type A;
    matrix_type W;
    matrix_type J;
    math_policy::compute_system_matrix(W,J,A);
  }
}

void (*single_precision_default_math_policy)()  = &(math_policy_compile_test< OpenTissue::mbd::default_ublas_math_policy<float>  > );
void (*double_precision_default_math_policy)() = &(math_policy_compile_test< OpenTissue::mbd::default_ublas_math_policy<double> > );
void (*single_precision_optimized_math_policy)()  = &(math_policy_compile_test< OpenTissue::mbd::optimized_ublas_math_policy<float>  > );
void (*double_precision_optimized_math_policy)() = &(math_policy_compile_test< OpenTissue::mbd::optimized_ublas_math_policy<double> > );
