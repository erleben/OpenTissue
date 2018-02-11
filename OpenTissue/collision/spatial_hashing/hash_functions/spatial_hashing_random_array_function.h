#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_RANDOM_ARRAY_FUNCTION_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_RANDOM_ARRAY_FUNCTION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <vector>
#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {
    /**
    * Hash Function suggested by J.A. Bærentzen.
    *
    *
    * return int((randoms1[key[0]] >> (key[1]&0x0f)) +
    *      (randoms2[key[1]] >> (key[2]&0x0f)) +
    *      (randoms3[key[2]] >> (key[0]&0x0f))) & (use_size-1);
    */
    class RandomArrayHashFunction
    {
    protected:

      size_t m_size;

      std::vector<size_t> m_random1;
      std::vector<size_t> m_random2;
      std::vector<size_t> m_random3;

    public:

      RandomArrayHashFunction()                             {        resize(1000);      }
      explicit RandomArrayHashFunction(size_t size)      {        resize(size);      }

    public:

      size_t operator()( int i,int j, int k )
      {
        if(i<0)  i = -i;
        if(j<0)  j = -j;
        if(k<0)  k = -k;

        size_t ii = i;
        size_t jj = j;
        size_t kk = k;

        if(ii >= m_size)  ii = ii % m_size;
        if(jj >= m_size)  jj = jj % m_size;
        if(kk >= m_size)  kk = kk % m_size;

        size_t hash_key = (
          (m_random1[ii] >> (jj&0x0f)) +
          (m_random2[jj] >> (kk&0x0f)) + 
          (m_random3[kk] >> (ii&0x0f))
          ) & (m_size-1u);

        assert( hash_key >= 0 || !"RandomArrayHashFunction::operator(): hash_key must be non-negative");
        assert( hash_key < m_size || !"RandomArrayHashFunction::operator(): hash_key must smaller than size");
        return hash_key;
      }

      void resize(size_t new_size)
      {
        using std::floor;

        math::Random<double> r;

        size_t max_val = ~0u;

        m_size = new_size ;
        m_random1.resize(m_size);
        m_random2.resize(m_size);
        m_random3.resize(m_size);
        for(size_t i=0;i<m_size;++i)
        {
          m_random1[i] = static_cast<size_t>( floor(max_val * r()) );
          m_random2[i] = static_cast<size_t>( floor(max_val * r()) );
          m_random3[i] = static_cast<size_t>( floor(max_val * r()) );
        }
      }

      size_t size() const { return m_size; }
    };

  } // namespace spatial_hashing

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_RANDOM_ARRAY_FUNCTION_H
#endif
