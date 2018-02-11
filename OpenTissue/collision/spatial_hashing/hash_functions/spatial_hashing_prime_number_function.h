#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_PRIME_NUMBER_FUNCTION_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_PRIME_NUMBER_FUNCTION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_prime_numbers.h>

#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {
    /**
    * Hash Function suggested by Teschner et. al.
    */
    class PrimeNumberHashFunction
    {
    protected:

      size_t m_size;  ///< The number of hash cells.
      int m_p1;             ///< Large prime number.
      int m_p2;             ///< Large prime number.
      int m_p3;             ///< Large prime number.

    public:

      PrimeNumberHashFunction()
        : m_size(1000)
        , m_p1( 73856093 )
        , m_p2( 19349663 )
        , m_p3( 83492791 )
      { }

      PrimeNumberHashFunction(size_t size)
        : m_size(size)
        , m_p1( 73856093 )
        , m_p2( 19349663 )
        , m_p3( 83492791 )
      {
        assert( m_size > 0 );
      }

    public:

      /**
      * Hash Function.
      *
      * @param p    A discretized point (this is not verified by the implementation) identifying a unique grid cell.
      * @return     The index of a hash cell, which the grid cell should be mapped to.
      */
      size_t operator()( int i,int j, int k )
      {
        int hash_key = ( i * m_p1 ^ j * m_p2 ^ k * m_p3 ) % m_size;
        if ( hash_key < 0 )
          hash_key += m_size;
        assert( hash_key >= 0 );
        assert( static_cast<size_t>(hash_key) < m_size );
        return hash_key;
      }

      /**
      * The closest prime number is found and the size is set to this number.
      *
      * @param size     New size of hashtable.
      */
      void resize(size_t new_size)
      {
        m_size = math::prime_search( new_size );
      }

      size_t size()const { return m_size; }
    };

  } // namespace spatial_hashing
} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_PRIME_NUMBER_FUNCTION_H
#endif
