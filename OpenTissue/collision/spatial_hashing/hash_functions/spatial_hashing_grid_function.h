#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_GRID_FUNCTION_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_GRID_FUNCTION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {
    /**
    * Hash Function suggested by K. Erleben.
    *
    * Well it seems to work nice if one got a good nice even spread of objects...
    *
    */
    class GridHashFunction
    {
    protected:

      size_t m_size;
      int m_dx;

    public:

      GridHashFunction()               { resize(1000); }
      GridHashFunction(size_t size) { resize(size); }

    public:

      size_t operator()( int i,int j, int k )
      {
        while(i<0)  i += m_dx;
        while(j<0)  j += m_dx;
        while(k<0)  k += m_dx;
        if(i >= m_dx)  i = i % m_dx;
        if(j >= m_dx)  j = j % m_dx;
        if(k >= m_dx)  k = k % m_dx;

        int hash_key = (i*m_dx + j)*m_dx + k;
        hash_key = hash_key % m_size;

        assert( hash_key >= 0 );
        assert( static_cast<size_t>(hash_key) < m_size );
        return hash_key;
      }

      void resize(size_t new_size)
      {
        using std::floor;

        float b = static_cast<float>(new_size);
        float a = pow( b , 1.0f/3.0f);
        m_dx = static_cast<int>( floor(a + 0.5f) );
        if(m_dx < 2)   m_dx = 2;
        m_size = m_dx*m_dx*m_dx;
      }

      size_t size()const { return m_size; }
    };

  } // namespace spatial_hashing

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_FUNCTIONS_SPATIAL_HASHING_GRID_FUNCTION_H
#endif
