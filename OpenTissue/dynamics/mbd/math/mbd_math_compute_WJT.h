#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_WJT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_WJT_H
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
      * Multiply inverse mass matrix by transposed jacobian.
      *
      * @param W      The inverse mass matrix.
      * @param J      The jacobian matrix.
      * @param WJT    Upon return this argument holds the value of prod(W,trans(J)),
      *               where W is the inverted mass matrix. This argument is assumed
      *               to be stored as prod(J,W) = trans( prod( W, trans(J) ) ). 
      *
      */
      template<typename matrix_type>
      void compute_WJT(matrix_type const & W, matrix_type const & J, matrix_type & WJT)
      {
        typedef typename matrix_type::value_type  real_type;
        typedef typename matrix_type::size_type   size_type;

        assert(W.size1()>0            || !"compute_WJT(): W was empty");
        assert(W.size2()>0            || !"compute_WJT(): W was empty");
        assert(J.size1()>0            || !"compute_WJT(): J was empty");
        assert(J.size2()>0            || !"compute_WJT(): J was empty");
        assert(W.size2() == J.size2() || !"compute_WJT(): incorrect dimensions");

        // A small example of what is going on here!!
        //
        //     W1   0   0  0
        //      0  W2   0  0
        //W =   0   0  W3  0
        //      0   0   0 W4
        //
        //        A 0 B 0 
        //        0 C D 0 
        //   J =  E F 0 0 
        //        G 0 0 H 
        //        0 K 0 L 
        //
        //           A 0 E G 0 
        //   J^T  =  0 C F 0 K
        //           B D 0 0 0
        //           0 0 0 H L
        //
        //
        //           W1*A    0 W1*E W1*G    0 
        // W  J^T  =    0 W2*C W2*F    0 W2*K
        //           W3*B W3*D    0    0    0
        //              0    0    0 W4*H W4*L
        //
        //
        //  W^T = W
        //
        //  (W J^T)^T = (J W)^T
        //
        //
        //
        //                      | m                    |
        //                      |    m                 |
        //                      |       m              |
        //| j1 j2 j3 j4 j5 j6 | |          I00 I01 I02 |
        //                      |          I10 I11 I12 |
        //                      |          I20 I21 I22 |
        //

        WJT.resize(J.size1(), J.size2(),false);

        size_type k[2];
        for (size_type i = 0; i < J.size1 (); ++i)
        {
          size_type begin = J.index1_data()[i];
          size_type end = J.index1_data()[i+1];

          assert((end-begin)==12 || !"compute_WJT(): J cannot be a jacobian matrix?");

          k[0]  = begin;
          k[1]  = end-6;
          for(size_type body=0;body<2;++body)
          {
            size_type j = k[body];

            real_type const & j1 = J.value_data()[j];
            size_type const & c1 = J.index2_data()[j];
            ++j;

            real_type const & j2 = J.value_data()[j];
            size_type const & c2 = J.index2_data()[j];
            ++j;

            real_type const & j3 = J.value_data()[j];
            size_type const & c3 = J.index2_data()[j];
            ++j;

            real_type const & j4 = J.value_data()[j];
            size_type const & c4 = J.index2_data()[j];
            ++j;

            real_type const & j5 = J.value_data()[j];
            size_type const & c5 = J.index2_data()[j];
            ++j;

            real_type const & j6 = J.value_data()[j];
            size_type const & c6 = J.index2_data()[j];

            real_type const & m   = W( c1, c1 );
            real_type const & i00 = W( c4, c4 );
            real_type const & i01 = W( c4, c5 );
            real_type const & i02 = W( c4, c6 );
            real_type const & i11 = W( c5, c5 );
            real_type const & i12 = W( c5, c6 );
            real_type const & i22 = W( c6, c6 );

            assert(is_number(j1) || !"compute_WJT(): not a number encountered");
            assert(is_number(j2) || !"compute_WJT(): not a number encountered");
            assert(is_number(j3) || !"compute_WJT(): not a number encountered");
            assert(is_number(j4) || !"compute_WJT(): not a number encountered");
            assert(is_number(j5) || !"compute_WJT(): not a number encountered");
            assert(is_number(j6) || !"compute_WJT(): not a number encountered");

            assert(is_number(m) || !"compute_WJT(): not a number encountered");
            assert(is_number(i00) || !"compute_WJT(): not a number encountered");
            assert(is_number(i01) || !"compute_WJT(): not a number encountered");
            assert(is_number(i02) || !"compute_WJT(): not a number encountered");
            assert(is_number(i11) || !"compute_WJT(): not a number encountered");
            assert(is_number(i12) || !"compute_WJT(): not a number encountered");
            assert(is_number(i22) || !"compute_WJT(): not a number encountered");

            WJT(i,c1) = j1*m;
            WJT(i,c2) = j2*m;
            WJT(i,c3) = j3*m;
            WJT(i,c4) = j4*i00 + j5*i01 + j6*i02;
            WJT(i,c5) = j4*i01 + j5*i11 + j6*i12;
            WJT(i,c6) = j4*i02 + j5*i12 + j6*i22;

            assert(is_number(WJT(i,c1)) || !"compute_WJT(): not a number encountered");
            assert(is_number(WJT(i,c2)) || !"compute_WJT(): not a number encountered");
            assert(is_number(WJT(i,c3)) || !"compute_WJT(): not a number encountered");
            assert(is_number(WJT(i,c4)) || !"compute_WJT(): not a number encountered");
            assert(is_number(WJT(i,c5)) || !"compute_WJT(): not a number encountered");
            assert(is_number(WJT(i,c6)) || !"compute_WJT(): not a number encountered");
          }
          assert((WJT.index1_data()[i+1]-WJT.index1_data()[i])==12 || !"compute_WJT(): WJT did not have jacobian format?");
        }
      }

    } // end  namespace math
  } // end  namespace mbd
} // end namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_WJT_H
#endif
