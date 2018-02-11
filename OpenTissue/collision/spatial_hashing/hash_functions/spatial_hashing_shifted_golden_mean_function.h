#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_SHIFTED_GOLDEN_MEAN_FUNCTION_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_SHIFTED_GOLDEN_MEAN_FUNCTION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {
    /**
    * Hash Function suggested by Robert Bridson.
    *
    *
    * For a single 32-bit size_teger k:
    *
    *    size_t hash(size_t k)  { return k*2654435769u; }
    *
    * This is based on shifting the golden-mean to fill 32 bits, from Knuth. Then
    * for a triple of 32-bit integers (i,j,k):
    *
    *    size_t hash(int i, int j, int k){ return i ^ hash(j ^ hash(k)); }
    *
    * The hash function for 3 integers generalizes to any number of
    * integers, and 8-, 16-, or 64-bit quantities are just as easy to deal with,
    * using a different multiplier in the core hash function)
    */
    class ShiftedGoldenMeanHashFunction
    {
    protected:

      size_t m_size;

    public:

      ShiftedGoldenMeanHashFunction()
        : m_size(1000)
      {};

      ShiftedGoldenMeanHashFunction(size_t size)
        : m_size(size)
      {};

    private:

      size_t hash(size_t k)  { return k*2654435769u; };

    public:

      size_t operator()( int i,int j, int k )
      {
        int hash_key =  i ^ hash(j ^ hash(k));
        hash_key = hash_key % m_size;
        assert( hash_key >= 0 );
        assert( static_cast<size_t>(hash_key) < m_size );
        return hash_key;
      }

      void resize(size_t new_size) { m_size = new_size ; }
      size_t size()const { return m_size; }
    };

  } // namespace spatial_hashing

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_SHIFTED_GOLDEN_MEAN_FUNCTION_H
#endif
