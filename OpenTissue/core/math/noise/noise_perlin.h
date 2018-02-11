#ifndef OPENTISSUE_CORE_MATH_NOISE_NOISE_PERLIN_H
#define OPENTISSUE_CORE_MATH_NOISE_NOISE_PERLIN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>  //--- Needed for Random<>

namespace OpenTissue
{

  namespace noise
  {

    /**
    * Perlin Noise Function.
    * This class computes a real_type noise field in a 2-dimensional space.
    */
    template<typename real_type_>
    class PerlinNoise
    {
    public:

      typedef real_type_  real_type;

    private:

      int       * m_P;    ///< Permutation array
      real_type * m_G2x;  ///< X coordinate of gradient vectors
      real_type * m_G2y;  ///< Y coordinate of gradient vectors

      real_type * m_G3x;  ///< X coordinate of gradient vectors
      real_type * m_G3y;  ///< Y coordinate of gradient vectors
      real_type * m_G3z;  ///< Z coordinate of gradient vectors

    public:

      PerlinNoise()
      {
        math::Random<real_type> random(0.0,1.0);

        //--- Generate 2D gradient vectors
        m_G2x = new real_type[256];
        m_G2y = new real_type[256];
        for(int i=0;i<256;++i)
        {
          real_type r2= static_cast<real_type>(0.0);
          do
          {
            m_G2x[i] = random() - 0.5;
            m_G2y[i] = random() - 0.5;
            r2 = m_G2x[i]*m_G2x[i] + m_G2y[i]*m_G2y[i];
          }
          while(!(r2>0));//--- r2==0
          real_type r = static_cast<real_type>( std::sqrt(r2) );
          m_G2x[i] /= r;
          m_G2y[i] /= r;
        }

        //--- Generate 3D gradient vectors
        m_G3x = new real_type[256];
        m_G3y = new real_type[256];
        m_G3z = new real_type[256];
        for(int i=0;i<256;++i)
        {
          real_type r2= static_cast<real_type>(0.0);
          do
          {
            m_G3x[i] = random() - 0.5;
            m_G3y[i] = random() - 0.5;
            m_G3z[i] = random() - 0.5;
            r2 = m_G3x[i]*m_G3x[i] + m_G3y[i]*m_G3y[i] + m_G3z[i]*m_G3z[i];
          }
          while(!(r2>0));//--- r2==0

          real_type r = static_cast<real_type>( std::sqrt(r2) );
          m_G3x[i] /= r;
          m_G3y[i] /= r;
          m_G3z[i] /= r;
        }

        //--- Generate permutation array
        m_P = new int[256];
        for(int i=0;i<256;++i)
          m_P[i] = i;
        for(int i=0;i<256;++i)
        {
          int j = static_cast<int>(  random()*255.0  );
          int s = m_P[i];
          m_P[i] = m_P[j];
          m_P[j] = s;
        }
      }


      ~PerlinNoise(void)
      {
        if(m_G2x)
          delete [] m_G2x;
        if(m_G2y)
          delete [] m_G2y;
        if(m_G3x)
          delete [] m_G3x;
        if(m_G3y)
          delete [] m_G3y;
        if(m_G3z)
          delete [] m_G3z;
        if(m_P)
          delete [] m_P;
      }

      /**
      * Compute Noise value at specified location.
      * For details on the noise computation see comments on the 3D version.
      *
      * @param x       The x coordinate.
      * @param y       The y coordinate.
      *
      * @return        The noise value at the specified location.
      */
      real_type operator()(real_type const & x,real_type const & y)const
      {
        int gridLowX  = static_cast<int>(std::floor(x) );
        int gridHighX = gridLowX + 1;
        int gridLowY  = static_cast<int>(std::floor(y) );
        int gridHighY = gridLowY + 1;

        real_type g0x = m_G2x[ (gridLowX  + m_P[ gridLowY  & 0xFF] ) & 0xFF ];
        real_type g0y = m_G2y[ (gridLowX  + m_P[ gridLowY  & 0xFF] ) & 0xFF ];
        real_type g1x = m_G2x[ (gridHighX + m_P[ gridLowY  & 0xFF] ) & 0xFF ];
        real_type g1y = m_G2y[ (gridHighX + m_P[ gridLowY  & 0xFF] ) & 0xFF ];
        real_type g2x = m_G2x[ (gridHighX + m_P[ gridHighY & 0xFF] ) & 0xFF ];
        real_type g2y = m_G2y[ (gridHighX + m_P[ gridHighY & 0xFF] ) & 0xFF ];
        real_type g3x = m_G2x[ (gridLowX  + m_P[ gridHighY & 0xFF] ) & 0xFF ];
        real_type g3y = m_G2y[ (gridLowX  + m_P[ gridHighY & 0xFF] ) & 0xFF ];

        real_type t0x = x - gridLowX;
        real_type t0y = y - gridLowY;
        real_type t1x = x - gridHighX;
        real_type t1y = y - gridLowY;
        real_type t2x = x - gridHighX;
        real_type t2y = y - gridHighY;
        real_type t3x = x - gridLowX;
        real_type t3y = y - gridHighY;

        real_type dot0 = g0x*t0x + g0y*t0y;
        real_type dot1 = g1x*t1x + g1y*t1y;
        real_type dot2 = g2x*t2x + g2y*t2y;
        real_type dot3 = g3x*t3x + g3y*t3y;

        //--- Hermite spline interpolation
        real_type sx = (3.0 - 2.0*t0x)*t0x*t0x;
        real_type a =  dot0 + sx*(dot1-dot0);
        real_type b =  dot3 + sx*(dot2-dot3);

        real_type sy = (3.0 - 2.0*t0y)*t0y*t0y;
        real_type z =  a+sy*(b-a);

        return z;
      }

      /**
      * Compute 3D Noise at the specified location.
      * The noise computation is done as follows:
      *
      * 1) A cube lattice is setup.
      *
      * 2) At each of the eight corners a random
      *    normalized gradient vector exist.
      *
      * 3) A direction vector from each corner the
      *    to the specified location is computed (and
      *    stored in the temporaries).
      *
      * 4) Now a ``noise'' value is computed at each of
      *    the eight corners, by taking the dot product
      *    between computed direction vector at that
      *    corner and the gradient at that corner.
      *
      * 5) Finally the noise value at the specified location
      *    is computed by trilinear hermite interpolation.
      *
      * This will create a smooth looking noise function.
      *
      * @param x       The x coordinate
      * @param y       The y coordinate
      * @param z       The z coordinate
      *
      * @return        The noise value at the specified location.
      */
      real_type operator()(real_type const & x,real_type const & y,real_type const & z)const
      {
        int gridLowX  = static_cast<int>(std::floor(x) );
        int gridHighX = gridLowX + 1;
        int gridLowY  = static_cast<int>(std::floor(y) );
        int gridHighY = gridLowY + 1;
        int gridLowZ  = static_cast<int>(std::floor(z) );
        int gridHighZ = gridLowZ + 1;

        //--- Obtain gradient vectors at eight corners
        real_type g000x = m_G3x[ (gridLowX   + m_P[ (gridLowY  + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g000y = m_G3y[ (gridLowX   + m_P[ (gridLowY  + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g000z = m_G3z[ (gridLowX   + m_P[ (gridLowY  + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g001x = m_G3x[ (gridLowX   + m_P[ (gridLowY  + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g001y = m_G3y[ (gridLowX   + m_P[ (gridLowY  + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g001z = m_G3z[ (gridLowX   + m_P[ (gridLowY  + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g010x = m_G3x[ (gridLowX   + m_P[ (gridHighY + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g010y = m_G3y[ (gridLowX   + m_P[ (gridHighY + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g010z = m_G3z[ (gridLowX   + m_P[ (gridHighY + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g011x = m_G3x[ (gridLowX   + m_P[ (gridHighY + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g011y = m_G3y[ (gridLowX   + m_P[ (gridHighY + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g011z = m_G3z[ (gridLowX   + m_P[ (gridHighY + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g100x = m_G3x[ (gridHighX  + m_P[ (gridLowY  + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g100y = m_G3y[ (gridHighX  + m_P[ (gridLowY  + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g100z = m_G3z[ (gridHighX  + m_P[ (gridLowY  + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g101x = m_G3x[ (gridHighX  + m_P[ (gridLowY  + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g101y = m_G3y[ (gridHighX  + m_P[ (gridLowY  + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g101z = m_G3z[ (gridHighX  + m_P[ (gridLowY  + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g110x = m_G3x[ (gridHighX  + m_P[ (gridHighY + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g110y = m_G3y[ (gridHighX  + m_P[ (gridHighY + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g110z = m_G3z[ (gridHighX  + m_P[ (gridHighY + m_P[ gridLowZ  & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g111x = m_G3x[ (gridHighX  + m_P[ (gridHighY + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g111y = m_G3y[ (gridHighX  + m_P[ (gridHighY + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];
        real_type g111z = m_G3z[ (gridHighX  + m_P[ (gridHighY + m_P[ gridHighZ & 0xFF ] ) & 0xFF ] ) & 0xFF ];

        //--- Obtain direction vectors from eight corners to the point
        real_type t000x = (x - gridLowX);
        real_type t000y = (y - gridLowY);
        real_type t000z = (z - gridLowZ);
        real_type t001x = (x - gridLowX);
        real_type t001y = (y - gridLowY);
        real_type t001z = (z - gridHighZ);
        real_type t010x = (x - gridLowX);
        real_type t010y = (y - gridHighY);
        real_type t010z = (z - gridLowZ);
        real_type t011x = (x - gridLowX);
        real_type t011y = (y - gridHighY);
        real_type t011z = (z - gridHighZ);
        real_type t100x = (x - gridHighX);
        real_type t100y = (y - gridLowY);
        real_type t100z = (z - gridLowZ);
        real_type t101x = (x - gridHighX);
        real_type t101y = (y - gridLowY);
        real_type t101z = (z - gridHighZ);
        real_type t110x = (x - gridHighX);
        real_type t110y = (y - gridHighY);
        real_type t110z = (z - gridLowZ);
        real_type t111x = (x - gridHighX);
        real_type t111y = (y - gridHighY);
        real_type t111z = (z - gridHighZ);

        //--- Compute dot products
        real_type dot000 = g000x*t000x + g000y*t000y + g000z*t000z;
        real_type dot001 = g001x*t001x + g001y*t001y + g001z*t001z;
        real_type dot010 = g010x*t010x + g010y*t010y + g010z*t010z;
        real_type dot011 = g011x*t011x + g011y*t011y + g011z*t011z;
        real_type dot100 = g100x*t100x + g100y*t100y + g100z*t100z;
        real_type dot101 = g101x*t101x + g101y*t101y + g101z*t101z;
        real_type dot110 = g110x*t110x + g110y*t110y + g110z*t110z;
        real_type dot111 = g111x*t111x + g111y*t111y + g111z*t111z;

        //--- Hermite spline interpolation
        real_type sx = ( 3.0 - 2.0*t000x )*t000x*t000x;
        real_type sy = ( 3.0 - 2.0*t000y )*t000y*t000y;
        real_type sz = ( 3.0 - 2.0*t000z )*t000z*t000z;

        real_type x00 = dot000 + sx*(dot100 - dot000);
        real_type x01 = dot001 + sx*(dot101 - dot001);
        real_type x10 = dot010 + sx*(dot110 - dot010);
        real_type x11 = dot011 + sx*(dot111 - dot011);
        real_type y0 =  x00 + sy*(x10-x00);
        real_type y1 =  x01 + sy*(x11-x01);
        real_type w =  y0 + sz*(y1-y0);
        return w;
      }

    };

  }  // namespace noise

} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_NOISE_NOISE_PERLIN_H
#endif
