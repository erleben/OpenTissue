#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_SOLVERS_MBD_PROJECTED_GAUSS_SEIDEL_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_SOLVERS_MBD_PROJECTED_GAUSS_SEIDEL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_ncp_solver_interface.h>
#include <OpenTissue/dynamics/mbd/solvers/mbd_merit.h>
#include <OpenTissue/core/math/math_is_number.h>

namespace OpenTissue
{
  namespace mbd
  {

    template<  typename math_policy  >
    class ProjectedGaussSeidel 
      : public NCPSolverInterface<math_policy>
    {
    protected:

      typedef typename math_policy::value_traits        value_traits;
      typedef typename math_policy::real_type           real_type;
      typedef typename math_policy::size_type           size_type;
      typedef typename math_policy::matrix_type         matrix_type;
      typedef typename math_policy::system_matrix_type  system_matrix_type;
      typedef typename math_policy::vector_type         vector_type;
      typedef typename math_policy::idx_vector_type     idx_vector_type;

    protected:

      size_type            m_iterations;    ///< Maximum allowed number of iterations, default value is 5.
      bool                 m_profiling;     ///< Boolean flag indicating whether profiling of the solver is turned on or off. Default value is false.
      vector_type          m_theta;         ///< vector used for profiling. The i'th entry stores the value of the merit-function after the i'th iteration of the solver.
      system_matrix_type   m_A;

    public:

      void set_max_iterations(size_type value)
      {
        assert(value>0 || !"ProjectedGaussSeidel::set_max_iterations(): value must be positive");
        m_iterations = value;
      }

      bool       & profiling()       { return m_profiling; }
      bool const & profiling() const { return m_profiling; }

      vector_type const & theta() const { return m_theta; }


      real_type get_accuracy() const { return value_traits::zero(); } // Oups not implemented!
      size_t    get_iteration() const { return 0; }                   // Oups not implemented!


    public:

      ProjectedGaussSeidel()
        : m_iterations(5)
        , m_profiling(false)
      {}

      virtual ~ProjectedGaussSeidel(){}

    public:

      void run(
          matrix_type const & J
        , matrix_type const & W
        , vector_type const & gamma
        , vector_type const & b
        , vector_type & lo
        , vector_type & hi
        , idx_vector_type const & pi
        , vector_type const & mu
        , vector_type & x
        )  
      {
        using std::fabs;

        if(this->profiling())
          math_policy::resize(m_theta,m_iterations);

        size_type m;
        math_policy::get_dimension(b,m);

        if(m==0)
          return;

        math_policy::compute_system_matrix(W, J, m_A);

        math_policy::init_system_matrix(m_A,x);

        //
        // In the non-regularized case we have
        //
        //    A x + b = 0
        //   (U + D + L) x = - b 
        //    D x = - b -U x - L x
        //    D x = - b -U x - L x - Dx + Dx
        //    D x = - b - A x  + Dx
        //      x = D^{-1}( - b - A x) + x
        //
        // In the regularized case A have been redefined as 
        //
        //   A' = A + gamma
        //
        // thus
        //
        //     A' = L + D' + U where D' = D + gamma
        //
        // which becomes
        //
        //      x = D'^{-1}( - b - A' x) + x
        //
        // Bu substitution we have
        // 
        //      x = (D+gamma)^{-1}( - b - A x - gamma x) + x
        //
        for (size_type k = 0; k < m_iterations; ++k)
        {
          for (size_type i = 0; i < m; ++ i)
          {
            real_type new_x  = - b(i);
            new_x -= math_policy::row_prod(m_A,i,x);

            assert(is_number(gamma(i))             || !"ProjectedGaussSeidel::run(): not a number encountered");
            assert(gamma(i)>= value_traits::zero() || !"ProjectedGaussSeidel::run(): gamma(i) was less than 0");

            if(gamma(i) > value_traits::zero())
            {
              // Take regularization term lineary to zero
              real_type alpha = value_traits::one()*(m_iterations-1-k)/(m_iterations-1);

              assert(is_number(alpha)                || !"ProjectedGaussSeidel::run(): not a number encountered");
              assert(alpha<= value_traits::one()     || !"ProjectedGaussSeidel::run(): alpha was greater than 1");
              assert(alpha>= value_traits::zero()    || !"ProjectedGaussSeidel::run(): alpha was less than 0");

              new_x -= gamma(i)*alpha*x(i);
              new_x /= m_A(i,i) + gamma(i)*alpha;
            }
            else
            {
              assert(m_A(i,i)>0 || m_A(i,i)<0 || !"ProjectedGaussSeidel::run(): diagonal entry is zero?");
              new_x /= m_A(i,i);
            }

            new_x += x(i);

            assert(is_number(new_x) || !"ProjectedGaussSeidel::run(): not a number encountered");

            size_type j = pi(i);
            if (j < m )
            {
              assert(is_number(mu(i)) || !"ProjectedGaussSeidel::run(): not a number encountered");
              hi(i) = fabs(mu(i)*x(j));
              lo(i) = - hi(i);
            }

            assert(lo(i)<= value_traits::zero()  || !"ProjectedGaussSeidel::run(): lower limit was positive");
            assert(hi(i)>= value_traits::zero()  || !"ProjectedGaussSeidel::run(): upper limit was negative");

            real_type old_x = x(i);
            if(new_x < lo(i))
              x(i) = lo(i);
            else if(new_x > hi(i))
              x(i) = hi(i);
            else
              x(i) = new_x;

            real_type dx = x(i)-old_x;

            math_policy::update_system_matrix(m_A,i,dx);

            assert(is_number(x(i)) || !"ProjectedGaussSeidel::run(): not a number encountered");
          }

          if(this->profiling())
            m_theta(k) = mbd::merit(m_A,x,b,lo,hi,math_policy());
        }
      }

    };

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_SOLVERS_MBD_PROJECTED_GAUSS_SEIDEL_H
#endif
