#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_BACKSOLVE_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_BACKSOLVE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <boost/cast.hpp>             // needed for boost::numeric_cast

#include <cassert>
#include <cmath>                      // needed for std::fabs

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {
        /**
        * Solve Linear Upper Triangular Problem.
        *
        * @param m      The size of the problem.
        * @param A      A square upper triangular matrix of dimension at least m \times m.
        * @param x      Upon return this argument holds the result of x = A^{-1} b. Note x is of dimension b upon return.
        * @param b      The right hand side vector of A x = b.  Note b is of dimension at least m.
        */
        template<typename size_type, typename matrix_type, typename vector_type>
        inline void backsolve ( 
          size_type m
          , matrix_type const & A
          , vector_type & x
          , vector_type const & b
          )
        {
          using std::fabs;

          typedef typename vector_type::value_type value_type;

          assert(m>0            || !"backsolve: m too small");
          assert(b.size() >= m  || !"backsolve: b too small");
          assert(A.size1() >= m || !"backsolve: A too small");
          assert(A.size2() >= m || !"backsolve: A too small");

          // Try to solve
          //
          //  |x_1|       | A_11      ...    A_1m | | b_1 |
          //  |x_2|       | 0    A_22 ...    A_2m | | b_2 |
          //  |.  |    =  | .                  .  | |  .  |
          //  |.  |       | .        ..        .  | |  .  |
          //  |x_m|       | 0        ..    0 A_mm | | b_m |
          //
          // This can be done using a backsolve traversal
          //
          //   x_m = b_m / A_mm
          //   x_{m-1} = ( b_{m-1} - A_{m-1,m} x_m ) / A_(m-1,m-1)
          //   .
          //   .
          //   .
          //   x_i = ( b_i - ( sum_{j=1}^{i-1} A_{i,j} X_j ) ) / A_(i,i)
          //
          // This can be implented a little differently, start by setting x = b, then
          // whenever we know the value of x_i we compute
          //
          //    x_j = x_j - A_{j,i} x_i  for all j<i
          //
          // Next we can compute x_(i-1) simply by  x_{i-1} /= A_{i-1,i-1}, and so on.
          //
          x.resize(b.size(), false);
          x = b;

          int k = ::boost::numeric_cast<int>(m);

          for ( int i = k - 1; i >= 0; --i)
          {
            assert( fabs(A( i, i ))> math::working_precision<value_type>()  || !"back_solve(): A is near singular");
            x [ i ] /= A( i, i );
            assert( is_number( x[i] ) || !"back_solve(): x_i is not a number");
            for ( int j = i - 1; j >= 0; --j)
              x [ j ] -= A ( j, i ) * x [ i ];
          }
        }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_BACKSOLVE_H
#endif
