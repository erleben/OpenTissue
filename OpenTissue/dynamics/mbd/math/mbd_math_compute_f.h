#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_F_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_F_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <OpenTissue/dynamics/mbd/math/mbd_math_update_f.h>

#include <cassert>

namespace OpenTissue
{
  namespace mbd
  {
    namespace math
    {

      /**
      * Compute f-vector.
      * The f-vector is defined as
      *
      *  f = prod( prod(W, trans(J) ), x)
      *
      * We can write x as
      *
      *  e_1 x_1 + ... +e_i x_i +  ... + e_n x_n
      *
      * So f becomes
      *
      *  f = W J^T ( sum_i^n e_i x_i)
      *
      * From this we see that we can compute f by invoking update_f n-times.
      *
      *
      * @param WJT    The value of prod(W, trans(J)) stored as prod(J,W).
      * @param x
      * @param f
      */
      template<typename matrix_type, typename vector_type>
      void compute_f(matrix_type const & WJT,vector_type const & x,vector_type & f)
      {
        typedef typename matrix_type::size_type  size_type;

        assert( WJT.size1() >0          || !"compute_f(): WJT was empty?");
        assert( WJT.size2() >0          || !"compute_f(): WJT was empty?");
        assert( WJT.size1() == x.size() || !"compute_f(): Incorrect dimensions");

        f.resize( WJT.size2(), false );
        f.clear();

        for (size_type i = 0;i < x.size() ; ++i)
          update_f(WJT,i,x(i),f);
      }


    } // end  namespace math
  } // end  namespace mbd
} // end namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_COMPUTE_F_H
#endif
