#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_UPDATE_F_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_UPDATE_F_H
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
      * Update f-vector.
      * Let
      *
      *  f = W J^T x
      *
      * Compute new f-value, f', when  x_i = x_i + dx, that is
      *
      *  f' = W J^T (x + e_i dx)
      *
      * where e_i is the i'th euclidean basis vector. Now
      *
      *  f' = f  + W J^T ( e_i dx)
      *  f' = f + column_i(W J^T) dx
      *  f' = f + row_i(J W) dx
      *
      * The last equation is used to update the f-vector to its new value.
      *
      * @param WJT    The value of prod(W, trans(J)) stored as prod(J,W).
      * @param i
      * @param dx
      * @param f
      *
      */
      template<typename matrix_type, typename size_type, typename real_type, typename vector_type>
      void update_f(
        matrix_type const & WJT
        , size_type   const & i
        , real_type   const & dx
        , vector_type       & f
        )
      {
        assert( WJT.size1() >0         || !"update_f(): WJT was empty?"      );
        assert( WJT.size2() >0         || !"update_f(): WJT was empty?"      );
        assert(i < WJT.size1()         || !"update_f(): incorrect i-value"   );
        assert(f.size() == WJT.size2() || !"update_f(): incorrect dimensions");
        assert(is_number(dx)           || !"update_f(): not a number encountered");

        size_type  begin = WJT.index1_data () [i];
        size_type  end   = WJT.index1_data () [i + 1];
        assert( (end-begin)==12 || !"update_f(): incorrect format of WJT");
        for (size_type j = begin; j < end; ++j)
        {
          assert(is_number(WJT.value_data()[j]) || !"update_f(): not a number encountered");
          f(WJT.index2_data()[j]) += WJT.value_data()[j] * dx;
          assert( is_number(  f(WJT.index2_data()[j]) ) || !"update_f(): not a number encountered");
        }
      }



    } // end  namespace math
  } // end  namespace mbd
} // end namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_UPDATE_F_H
#endif
