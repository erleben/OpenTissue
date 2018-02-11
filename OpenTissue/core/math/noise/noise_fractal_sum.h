#ifndef OPENTISSUE_CORE_MATH_NOISE_NOISE_FRACTAL_SUM_H
#define OPENTISSUE_CORE_MATH_NOISE_NOISE_FRACTAL_SUM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/noise/noise_improved_perlin.h>


namespace OpenTissue
{

  namespace noise
  {

    /**
    * Fractal Sum Noise Function
    *
    *    sum_{i=0}^{octaves-1} \frac{noise(2^i x,2^i y,2^i z) }{ 2^i}
    */
    template<typename real_type_>
    class FractalSumNoise
    {
    public:

      typedef real_type_                        real_type;
      typedef ImprovedPerlinNoise<real_type>    noise_type;

    protected:

      int          m_octaves;  ///< Number of octaves.
      noise_type   m_noise;

    public:

      FractalSumNoise(int const & octaves)
        : m_octaves(octaves)
      { }

      real_type operator()(real_type const & x,real_type const & y)const
      {
        real_type sum = 0;
        for(int i=0;i<m_octaves;++i)
        {
          int scale = (1<<i);
          real_type fraction = (1.0/scale);
          sum += fraction*m_noise(scale*x,scale*y);
        }
        return sum;
      }

      real_type operator()(real_type const & x,real_type const & y,real_type const & z)const
      {
        real_type sum = 0;
        for(int i=0;i<m_octaves;++i)
        {
          int scale = (1<<i);
          real_type fraction = (1.0/scale);
          sum += fraction*m_noise(scale*x,scale*y,scale*z);
        }
        return sum;
      }

    };

  } // namespace noise

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_NOISE_NOISE_FRACTAL_SUM_H
#endif
