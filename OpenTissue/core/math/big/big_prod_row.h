#ifndef OPENTISSUE_CORE_MATH_BIG_PROD_ROW_H
#define OPENTISSUE_CORE_MATH_BIG_PROD_ROW_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Compute y_i = prod(row_i(A),x).
      *
      * @param A         A matrix
      * @param x         A vector to be multipled with the matrix.
      * @param i         The index of the i'th row value that should be computed.
      *
      * @return          The value of the product of the i'th row with x-vector.
      */
      template<typename T>
      inline T prod_row(
        boost::numeric::ublas::compressed_matrix<T> const & A
        , boost::numeric::ublas::vector<T> const & x
        , typename boost::numeric::ublas::vector<T>::size_type i
        )
      {
        typedef boost::numeric::ublas::vector<T> vector_type;
        typedef typename vector_type::size_type  size_type;
        typedef typename vector_type::value_type real_type;

        assert(A.size1()>0            || !"prod_row(): A was empty"            );
        assert(A.size2()>0            || !"prod_row(): A was empty"            );
        assert(A.size2() ==  x.size() || !"prod_row(): incompatible dimensions");
        assert(i < A.size1()          || !"prod_row(): incompatible dimensions");

        real_type value = real_type();

        //
        //  Example of compressed matrix format:
        //      
        //    0 x y 0
        //    0 0 z 0
        //    w 0 0 0
        //
        //   value_data: [ x y z w ]
        //
        //    Note this is simply all non-zero elements of the sparse matrix stored in a row-by-row wise manner.
        //
        //   index1_data: [ 0 2 3 ]     
        //
        //    Note that the number of elements is equal to the number of rows in the
        //    sparse matrix. Each element points to the index in value_data that
        //    corresponds to the starting element of the corresponding row.
        //
        //   index2_data: [ 1 2 2 0 ]     
        //
        //    Note this array have the same dimension as value_data. Each element in index2_data
        //    stores the corresponind column index of the matching element in value_data.
        //

        size_type row_end = A.filled1 () - 1;
        if(i>=row_end)
          return value;

        size_type begin = A.index1_data () [i];
        size_type end   = A.index1_data()[i + 1];
        for (size_type j = begin; j < end; ++j)
          value += A.value_data()[j] * x(  A.index2_data()[j]  );
        assert(is_number(value) || !"prod_row(): not a number encountered");
        return value;
      }

    } // end  namespace big
  } // end  namespace math
} // end namespace OpenTissue
// OPENTISSUE_CORE_MATH_BIG_PROD_ROW_H
#endif
