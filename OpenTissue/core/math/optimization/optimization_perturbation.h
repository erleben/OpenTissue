#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_PERTURBATION_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_PERTURBATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

        /**
        * Perturbation Function.
        * In the phd thesis of Billups a pertubed problem is created, by replacing a function f(x) with another function
        *
        *    f(\lambda,y,x) = f(x) + \lambda(x - y)
        *
        * where y is termed the center point and \lambda>0 is the perturbation value or simply the amount of perturbation.
        *
        * In the case of the a linear mixed complementarity problem we have f(x) = A x + b, so this implies
        *
        *  f(\lambda,y,x) = f(x) + \lambda(x - y)
        *                  = A x + b + \lambda(x - y)
        *                  = (A + \lambda I)x + ( b - \lambda y)
        *
        * We thus see that the perturbed problem corresponds to adding a postive value to
        * the diagonal of the coefficient matrix A, and modifying the right-hand-side by
        * subtracting a positive scalar multiple of the center point.
        *
        * Defining:
        *
        *   A^\prime = (A + \lambda I)
        *   b^\prime = ( b - \lambda y)
        *
        * We see that the perturbed problem is also a affine function of x as is the un-perturbed function.
        *
        *  f(\lambda,y,x) = A^\prime x + b^\prime
        *
        * This function computes the matrix and rhs-vector of the perturbed problem. Observe
        * that usually A is a symmetric positive semi-definite matrix. Thus A_prime is
        * going to be a symmetric positive definite matrix. In terms of the LCP book by Cottle,
        * Pang, and Stone this is termed a regularization (see page 442).
        *
        * @param y            The center point.
        * @param lambda       The perturbation value
        * @param A            Original coefficient matrix.
        * @param b            Original right-hand-side vector
        * @param A_prime      Upon return this argument holds the perturbed coefficient matrix.
        * @param b_prime      Upon return this argument holds the perturbed right-hand-side vector.
        */
        template<typename T>
        inline void perturbation(
          boost::numeric::ublas::vector<T> & y
          , T const & lambda
          , ublas::compressed_matrix<T> const & A
          , ublas::vector<T> const & b
          , ublas::compressed_matrix<T>  & A_prime
          , ublas::vector<T>  & b_prime
          )
        {
          size_t const m = A.size1();
          size_t const n = A.size2();

          assert(m>0           || !"perturbation(): A was empty");
          assert(n>0           || !"perturbation(): A was empty");
          assert(m==n          || !"perturbation(): A was not square");
          assert(b.size() == m || !"perturbation(): Incompatible dimension between A and b");
          assert(y.size() == m || !"perturbation(): Incompatible dimension between b and y");

          A_prime.resize(m,n,false);
          b_prime.resize(n);
          A_prime.assign( A );
          b_prime.assign( b );

          for(size_t i=0;i<m;++i)
          {
            A_prime(i,i) += lambda;
            b_prime(i) -= lambda*y(i);
          }
        }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_PERTURBATION_H
#endif
