#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_DIAGONAL_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_DIAGONAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <cassert>

namespace OpenTissue
{
  namespace mbd
  {
    namespace math
    {

      /**
      * Compute the diagonal of A.
      * That is we compute the diagonal of 
      * 
      *  noalias(A) = prod( J, matrix_type( prod( W, trans(J) ) ) )
      *
      * Exploiting that we already have pre-computed the term prod( W, trans(J) ).
      *
      * @param J      The jacobian matrix.
      * @param WJT    The value of prod(W,trans(J)), where W is the inverted
      *               mass matrix. This argument is assumed to be stored as
      *               prod(J,W) = trans( prod( W, trans(J) ) ). 
      * @param d      Upon return this vector contain the values of the
      *               diagonal of A. That is d(i) = A(i,i).
      */
      template<typename matrix_type,typename vector_type>
      void compute_diagonal(matrix_type const & J, matrix_type const & WJT, vector_type & d)
      {
        typedef typename matrix_type::size_type   size_type;
        typedef typename matrix_type::value_type  real_type;

        assert(J.size1()>0            || !"compute_diagonal(): J was empty");
        assert(J.size2()>0            || !"compute_diagonal(): J was empty");
        assert(J.size1()==WJT.size1() || !"compute_diagonal(): J and WJT did not match dimensions");
        assert(J.size2()==WJT.size2() || !"compute_diagonal(): J and WJT did not match dimensions");

        d.resize(J.size1(), false);

        for (size_type i = 0; i < J.size1 (); ++i)
        {
          size_type begin = J.index1_data()[i];
          size_type end   = J.index1_data()[i+1];

          assert( (end-begin)==12                                 || !"compute_diagonal(): incorrect format of J"                 );
          assert(  J.index1_data()[i] == WJT.index1_data()[i]     || !"compute_diagonal(): WJT and J did not have the same format");
          assert(  J.index1_data()[i+1] == WJT.index1_data()[i+1] || !"compute_diagonal(): WJT and J did not have the same format");

          size_type j = begin;
          real_type const & j1 = J.value_data()[j]; ++j;
          real_type const & j2 = J.value_data()[j]; ++j;
          real_type const & j3 = J.value_data()[j]; ++j;
          real_type const & j4 = J.value_data()[j]; ++j;
          real_type const & j5 = J.value_data()[j]; ++j;
          real_type const & j6 = J.value_data()[j];

          j = end-6;
          real_type const & j7  = J.value_data()[j]; ++j;
          real_type const & j8  = J.value_data()[j]; ++j;
          real_type const & j9  = J.value_data()[j]; ++j;
          real_type const & j10 = J.value_data()[j]; ++j;
          real_type const & j11 = J.value_data()[j]; ++j;
          real_type const & j12 = J.value_data()[j];

          j = begin;
          real_type const & wjt1 = WJT.value_data()[j]; ++j;
          real_type const & wjt2 = WJT.value_data()[j]; ++j;
          real_type const & wjt3 = WJT.value_data()[j]; ++j;
          real_type const & wjt4 = WJT.value_data()[j]; ++j;
          real_type const & wjt5 = WJT.value_data()[j]; ++j;
          real_type const & wjt6 = WJT.value_data()[j];

          j = end-6;
          real_type const & wjt7  = WJT.value_data()[j]; ++j;
          real_type const & wjt8  = WJT.value_data()[j]; ++j;
          real_type const & wjt9  = WJT.value_data()[j]; ++j;
          real_type const & wjt10 = WJT.value_data()[j]; ++j;
          real_type const & wjt11 = WJT.value_data()[j]; ++j;
          real_type const & wjt12 = WJT.value_data()[j];

          assert(is_number(j1) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j2) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j3) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j4) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j5) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j6) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j7) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j8) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j9) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j10) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j11) || !"compute_diagonal(): not a number encountered");
          assert(is_number(j12) || !"compute_diagonal(): not a number encountered");

          assert(is_number(wjt1) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt2) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt3) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt4) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt5) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt6) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt7) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt8) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt9) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt10) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt11) || !"compute_diagonal(): not a number encountered");
          assert(is_number(wjt12) || !"compute_diagonal(): not a number encountered");

          d(i) = j1*wjt1 + j2*wjt2 + j3*wjt3 + j4*wjt4 + j5*wjt5 + j6*wjt6 + j7*wjt7 + j8*wjt8 + j9*wjt9 + j10*wjt10 + j11*wjt11 + j12*wjt12;

          assert(is_number(d(i)) || !"compute_diagonal(): not a number encountered");
        }
      }

    } // end  namespace math
  } // end  namespace mbd
} // end namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_DIAGONAL_H
#endif
