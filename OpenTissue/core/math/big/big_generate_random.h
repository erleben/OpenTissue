#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_RANDOM_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_RANDOM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>


namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Generate Random Matrix.
      * This function is a convenience function that is usefull
      * for quick-and-dirty initialization.
      *
      * @param m  The number of rows in the resulting matrix.
      * @param n  The number of columns in the resulting matrix.
      * @param A  Upon return this argument holds an m-by-n matrix
      *           with a random value between zero and one in each
      *           entry of the matrix.
      */
      template<typename matrix_type>
      inline void generate_random( size_t const & m, size_t const & n,  matrix_type & A  )
      {
        typedef typename matrix_type::value_type                     value_type;
        typedef          OpenTissue::math::ValueTraits<value_type>   value_traits;

        Random<value_type> value(value_traits::zero(),value_traits::one());

        assert( m>0         || !"generate_random(): m was out of range");
        assert( n>0         || !"generate_random(): n was out of range");
  
        A.resize(m,n,false);
        A.clear();

        for(size_t i=0;i<A.size1();++i)
        {
          for(size_t j=0;j<A.size2();++j)
          {
            A(i,j) = value();
            assert( is_number( A(i,j) ) || !"generate_random(): not a number encountered");
          }
        }
      }


      /**
      * Generate Random Vector.
      * This function is a convenience function that is usefull
      * for quick-and-dirty initialization.
      *
      * @param n  The number of entries in the resulting vector.
      * @param A  Upon return this argument holds an n-vector
      *           with a random value between zero and one in each
      *           entry.
      */
      template<typename value_type>
      inline void generate_random( size_t const & n,  ublas::vector<value_type> & v  )
      {
        typedef ValueTraits<value_type> value_traits;

        Random<value_type> value(value_traits::zero(),value_traits::one());

        assert( n>0         || !"generate_random(): n was out of range");

        v.resize(n,false);
        v.clear();

        for(size_t i=0;i<n;++i)
        {
          v(i) = value();
          assert( is_number( v(i) ) || !"generate_random(): not a number encountered");
        }
      }


    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_RANDOM_H
#endif
