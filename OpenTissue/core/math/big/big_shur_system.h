#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_SHUR_SYSTEM_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_SHUR_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_prod.h>
#include <OpenTissue/core/math/big/big_prod_sub.h>

#include <OpenTissue/core/math/math_value_traits.h>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Compute Shur Equation.
      * WARNING: This function performs an inplace solve, which means that some of the arguments are used as temporaries. That is the values of the arguments are overridden.
      *
      * Given the partitioned matrix equation
      *
      *  | A_aa  A_ab | ´| x_a | = | rhs_a |
      *  | C       D  |  | x_b |   | rhs_b |
      *
      * This method solves for the solution using a Shur Complement. From the bottom row we derive
      *
      *    x_b  = inv(D)(  rhs_b - C x_a )          (*1)
      *
      * Substituting this into the top row and cleaning up one gets
      *
      *    A_aa x_a  +  A_ab inv(D)(  rhs_b - C x_a ) = rhs_a
      *    A_aa x_a  +  A_ab inv(D) rhs_b - A_ab inv(D) C x_a  = rhs_a
      *    (A_aa - A_ab inv(D) C) x_a    = rhs_a - A_ab inv(D) rhs_b
      *
      * Introducing the notation
      *
      *  M = (A_aa - A_ab inv(D) C)
      *  q = rhs_a - A_ab inv(D) rhs_b
      *
      * We discover the Shur Matrix equation
      *
      *  M x_a = q               (*2)
      *
      * The idea is to solve for x_a using (*2) and then substituing
      * into (*1) to find x_b.
      *
      * @param A_aa    The top-left most sub-block of the matrix equation. Overridden upon return.
      * @param A_ab    The top-right most sub-block of the matrix equation.
      * @param C       The bottom-left most sub-block of the matrix equation.
      * @param invD    The inverse of the bottom-right most sub-block of the matrix equation.
      * @param rhs_a   The top most sub-block of the right hand side vector.  Overridden upon return.
      * @param rhs_b   The bottom most sub-block of the right hand side vector.  Overridden upon return.
      * @param dx_a    Upon return this argument holds the top most sub-block of the solution vector.
      * @param dx_b    Upon return this argument holds the bottom most sub-block of the solution vector.
      * @param solve   A function that solves a linear system, A x = b. It must have the signature  void ()(compressed_matrix const & A,vector & x, vector const & b). 
      *
      */
      template<typename T, typename solver_function_type>
      inline void  shur_system( 
        ublas::compressed_matrix<T>         & A_aa
        , ublas::compressed_matrix<T> const & A_ab
        , ublas::compressed_matrix<T> const & C
        , ublas::compressed_matrix<T> const & invD
        , ublas::vector<T>        & rhs_a
        , ublas::vector<T>        & rhs_b
        , ublas::vector<T>        & dx_a
        , ublas::vector<T>        & dx_b
        , solver_function_type const & solve
        )
      {
        typedef          ublas::compressed_matrix<T>       matrix_type;
        typedef          ublas::vector<T>                  vector_type;
        typedef typename vector_type::size_type            size_type;
        typedef          T                                 real_type;
        typedef          OpenTissue::math::ValueTraits<T>  value_traits;

        size_type A = rhs_a.size();
        size_type B = rhs_b.size();

        dx_a.resize(A);
        dx_b.resize(B);

        if(A>0)
        {
          if(B>0)
          {
            // Compute right hand side of Shur system
            //
            //   q = rhs_a  - A_ab * inv(D) * rhs_b;
            //

            OpenTissue::math::big::prod(invD, rhs_b, dx_b);
            OpenTissue::math::big::prod(A_ab,  dx_b, dx_a);
            rhs_a -= dx_a;

            // Compute Shur Matrix
            //
            //  M  = A_aa   - A_ab * inv(D) * C;
            //
            // Yikes, this is pain-full! If we use an incremental
            // solver then we should rather use a functor capable
            // of evaluating M*x, that is:
            //
            // M*x = (A_aa *x)  - (A_ab * (inv(D) * (C * x))) = y
            //
            //ShurMatrixOperator<matrix_type,vector_type> M(A_aa,A_ab,D,C);

            // Right now we do the ugly thing and compute the
            // matrix products:-(
            matrix_type M1;
            matrix_type M2;                 
            M1.resize( B, A, false );
            M2.resize( A, A, false );

            ublas::noalias( M1 ) = ublas::sparse_prod<matrix_type>( invD, C);
            ublas::noalias( M2 ) = ublas::sparse_prod<matrix_type>( A_ab, M1);
            ublas::noalias( A_aa ) -= M2;
          }

          // Solve Shur System
          //
          //   dx_a = M^{-1} q
          //
          solve(A_aa, dx_a, rhs_a);

          if(B>0)
          {
            // Substitute dx_a into equation for dx_b
            //
            //   dx_b =  inv(D) * ( rhs_b - C*dx_a);
            //
            OpenTissue::math::big::prod_sub( C, dx_a, rhs_b);
          }
        }
        if(B>0)
        {
          OpenTissue::math::big::prod(invD, rhs_b, dx_b);
        }

      }

    } // namespace big
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_SHUR_SYSTEM_H
#endif
