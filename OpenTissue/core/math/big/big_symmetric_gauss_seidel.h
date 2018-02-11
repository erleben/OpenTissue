#ifndef OPENTISSUE_CORE_MATH_BIG_SYMMETRIC_GAUSS_SEIDEL_H
#define OPENTISSUE_CORE_MATH_BIG_SYMMETRIC_GAUSS_SEIDEL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_forward_gauss_seidel.h>
#include <OpenTissue/core/math/big/big_backward_gauss_seidel.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Symmetric Gauss-Seidel Iteration.
      * This function implements a single iteration of a symmetric
      * Gauss Seidel method. That is it performs a forward Gauss-Seidel
      * iteration followed by a backward Gauss-Seidel iteration.
      *
      * @param A       A matrix.
      * @param x       Upon return this argument holds the new value of iterate x.
      * @param b       The right hand side vector.
      * 
      */
      template<typename T>
      inline void symmetric_gauss_seidel(
        boost::numeric::ublas::compressed_matrix<T> const & A
        , boost::numeric::ublas::vector<T>       & x
        , boost::numeric::ublas::vector<T> const & b
        )
      {
        forward_gauss_seidel( A, x, b );
        backward_gauss_seidel( A, x, b );
      }

      /**
      * Symmetric Gauss-Seidel Solver.
      * This function is capable of performing several iterations of the symmetric Gauss Seidel iteration.
      *
      * @param A                A matrix.
      * @param x                At invokation this argument holds the initial value of x^0, Upon return this argument holds the new value of iterate.
      * @param b                The right hand side vector.
      * @param max_iterations   The maximum number of iterations that is allowed.
      * @param epsilon          A stopping threshold (currently not used).
      * @param iterations       Upon return this argument holds the number of used iterations.
      */
      template<typename T>
      inline void symmetric_gauss_seidel(
        boost::numeric::ublas::compressed_matrix<T> const & A
        , boost::numeric::ublas::vector<T>       & x
        , boost::numeric::ublas::vector<T> const & b
        , size_t                           const & max_iterations
        , size_t                                 & iterations
        )
      {
        if(max_iterations < 1)
          throw std::invalid_argument("symmetric_gauss_seidel(): max_iterations must be a positive number");

        iterations = 0;
        while(iterations<max_iterations)
        {
          ++iterations;
          symmetric_gauss_seidel(A,x,b);
        }
      }

      /**
      * Symmetric Gauss Seidel Functor.
      * This is a convenience class providing
      * a functor interface for all the free template functions.
      *
      * It is the intention that this will make it easier for
      * end-users to pass the appropriate version of the symmetric Gauss Seidel
      * method to other algorithms.
      */
      class SymmetricGaussSeidelFunctor
      {
      public:

        template<typename T>
        void operator()(
          boost::numeric::ublas::compressed_matrix<T> const & A
          , boost::numeric::ublas::vector<T>       & x
          , boost::numeric::ublas::vector<T> const & b
          , size_t                           const & max_iterations
          , size_t                                 & iterations
          )
        {
          symmetric_gauss_seidel(A,x,b,max_iterations,iterations);
        }

        template<typename T>
        void operator()(
          boost::numeric::ublas::compressed_matrix<T> const & A
          , boost::numeric::ublas::vector<T>       & x
          , boost::numeric::ublas::vector<T> const & b
          )
        {
          symmetric_gauss_seidel(A,x,b);
        }

      };

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_SYMMETRIC_GAUSS_SEIDEL_H
#endif
