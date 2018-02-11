#ifndef OPENTISSUE_CORE_MATH_NOISE_NOISE_IMPROVED_PERLIN_H
#define OPENTISSUE_CORE_MATH_NOISE_NOISE_IMPROVED_PERLIN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


namespace OpenTissue
{

  namespace noise
  {

    /**
    * Improved Perlin Noise Function.
    */
    template<typename real_type_>
    class ImprovedPerlinNoise
    {
    public:

      typedef real_type_  real_type;

    protected:

      int m_p[512];

    protected:

      real_type fade(real_type const & t) const
      {
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
      }

      real_type lerp(real_type const & t, real_type const & a, real_type const & b)const
      {
        return a + t * (b - a);
      }

      real_type grad(int hash, real_type const & x, real_type const & y, real_type const & z) const
      {
        int h = hash & 15;                         // CONVERT LO 4 BITS OF HASH CODE
        real_type u = h<8 ? x : y;                 // INTO 12 GRADIENT DIRECTIONS.
        real_type v = h<4 ? y : h==12||h==14 ? x : z;
        return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
      }

    public:

      ImprovedPerlinNoise()
      {
        static int m_permutation[256] = {
          151,160,137,91,90,15,
          131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
          190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
          88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
          77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
          102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
          135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
          5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
          223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
          129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
          251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
          49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
          138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
        };
        for (int i=0; i < 256 ; i++)
          m_p[256+i] = m_p[i] = m_permutation[i];
      }

      real_type operator()(real_type const &x, real_type const & y, real_type const & z) const
      {
        int X = static_cast<int>( std::floor(x) ) & 0xFF; // FIND UNIT CUBE THAT
        int Y = static_cast<int>( std::floor(y) ) & 0xFF; // CONTAINS POINT.
        int Z = static_cast<int>( std::floor(z) ) & 0xFF;
        real_type rx = x - static_cast<real_type>( std::floor(x) );  // FIND RELATIVE X,Y,Z
        real_type ry = y - static_cast<real_type>( std::floor(y) );  // OF POINT IN CUBE.
        real_type rz = z - static_cast<real_type>( std::floor(z) );
        real_type u = fade(rx);                                // COMPUTE FADE CURVES
        real_type v = fade(ry);                                // FOR EACH OF X,Y,Z.
        real_type w = fade(rz);

        int A  = m_p[X  ] + Y;
        int AA = m_p[A  ] + Z;
        int AB = m_p[A+1] + Z;      // HASH COORDINATES OF
        int B  = m_p[X+1] + Y;
        int BA = m_p[B  ] + Z;
        int BB = m_p[B+1] + Z;      // THE 8 CUBE CORNERS,

        return lerp(w, lerp(v, lerp(u, grad(m_p[AA  ], rx    , ry    , rz     ),  // AND ADD
                                       grad(m_p[BA  ], rx-1.0, ry    , rz     )), // BLENDED
                               lerp(u, grad(m_p[AB  ], rx    , ry-1.0, rz     ),  // RESULTS
                                       grad(m_p[BB  ], rx-1.0, ry-1.0, rz     ))),// FROM  8
                       lerp(v, lerp(u, grad(m_p[AA+1], rx    , ry    , rz-1.0 ),  // CORNERS
                                       grad(m_p[BA+1], rx-1.0, ry    , rz-1.0 )), // OF CUBE
                               lerp(u, grad(m_p[AB+1], rx    , ry-1.0, rz-1.0 ),
                                       grad(m_p[BB+1], rx-1.0, ry-1.0, rz-1.0 ))));
      }


      real_type operator()(real_type const &x, real_type const & y) const
      {
        return (*this)(x,y,0.);
      }

      real_type operator()(real_type const &x ) const
      {
        return (*this)(x,0.,0.);
      }

    };

  } // namespace noise

} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_NOISE_NOISE_IMPROVED_PERLIN_H
#endif
