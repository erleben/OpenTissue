#ifndef OPENTISSUE_CORE_MATH_BIG_PROD_SUB_H
#define OPENTISSUE_CORE_MATH_BIG_PROD_SUB_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <OpenTissue/core/math/big/io/big_matlab_write.h>

#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
       * Compute y -= prod(A,x)
       *
       * @param A    A compressed matrix.
       * @param x    A vector.
       * @param y    Upon return this argument holds the
       *             result subtracting the value of A times x to the
       *             current value of the argument.
       */
      template<typename T>
      inline void prod_sub(        
          boost::numeric::ublas::compressed_matrix<T> const & A
        , boost::numeric::ublas::vector<T> const & x
        , boost::numeric::ublas::vector<T>       & y
        )
      {
        typedef boost::numeric::ublas::vector<T> vector_type;
        typedef typename vector_type::size_type  size_type;
        typedef typename vector_type::value_type real_type;

        assert(A.size1()>0            || !"prod_sub(): A was empty"            );
        assert(A.size2()>0            || !"prod_sub(): A was empty"            );
        assert(A.size2() ==  x.size() || !"prod_sub(): incompatible dimensions");
        assert(A.size1() ==  y.size() || !"prod_sub(): incompatible dimensions");

        size_type row_end = A.filled1 () - 1;
        for (size_type i = 0; i < row_end; ++ i) 
        {
          size_type begin = A.index1_data () [i];
          size_type end   = A.index1_data () [i + 1];
          real_type t     = real_type();
          for (size_type j = begin; j < end; ++j)
            t += A.value_data()[j] * x(  A.index2_data()[j]   );
          y(i) -= t;
        }
      }

    } // end  namespace big
  } // end  namespace math
} // end namespace OpenTissue
// OPENTISSUE_CORE_MATH_BIG_PROD_SUB_H
#endif
