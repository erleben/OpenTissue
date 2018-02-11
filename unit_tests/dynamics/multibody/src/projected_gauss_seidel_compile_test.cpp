//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/math/mbd_default_math_policy.h>
#include <OpenTissue/dynamics/mbd/solvers/mbd_projected_gauss_seidel.h>
#include <OpenTissue/dynamics/mbd/math/mbd_optimized_ublas_math_policy.h>

#include <iostream>

template<typename math_policy>
void compile_test_pgs()
{
  using namespace OpenTissue::math::big;

  typedef typename math_policy::real_type                real_type;
  typedef typename math_policy::size_type                size_type;
  typedef typename math_policy::value_traits             value_traits;
  typedef typename math_policy::idx_vector_type          idx_vector_type;
  typedef typename math_policy::vector_type              vector_type;
  typedef typename math_policy::matrix_type              matrix_type;
  typedef typename math_policy::system_matrix_type       system_matrix_type;

  typedef typename OpenTissue::mbd::ProjectedGaussSeidel<math_policy> solver_type;

  solver_type solver;

  size_type i = 0;
  solver.set_max_iterations(i);
  solver.profiling() = true;

  std::cout << solver.theta() << std::endl;

  matrix_type J,W;
  vector_type gamma;
  vector_type b;
  vector_type lo;
  vector_type hi;
  idx_vector_type pi;
  vector_type mu;
  vector_type x;
  solver.run(J,W,gamma,b,lo,hi,pi,mu,x);
}

void (*single_precision_default_pgs)()  = &(compile_test_pgs< OpenTissue::mbd::default_ublas_math_policy<float>  > );
void (*double_precision_default_pgs)() = &(compile_test_pgs< OpenTissue::mbd::default_ublas_math_policy<double> > );

void (*single_precision_optimized_pgs)()  = &(compile_test_pgs< OpenTissue::mbd::optimized_ublas_math_policy<float>  > );
void (*double_precision_optimized_pgs)() = &(compile_test_pgs< OpenTissue::mbd::optimized_ublas_math_policy<double> > );
