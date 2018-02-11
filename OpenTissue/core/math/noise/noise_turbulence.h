#ifndef OPENTISSUE_CORE_MATH_NOISE_NOISE_TURBULENCE_H
#define OPENTISSUE_CORE_MATH_NOISE_NOISE_TURBULENCE_H
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
    * Turbulence.
    *
    *  F(x,y,z) =  \sum_{i=0}^{octaves-1}  \left| \frac{ noise(2^i x,2^i y,2^i z) }{ 2^i} \right|
    *
    */
    template<typename real_type_>
    class Turbulence
    {
    public:

      typedef real_type_                      real_type;
      typedef ImprovedPerlinNoise<real_type>  noise_type;

    protected:

      unsigned int m_octaves;
      noise_type   m_noise;

    public:

      /**
      *
      * @param pixel_size
      */
      Turbulence(unsigned int octaves)
        : m_octaves(octaves)
      { }

      real_type operator()(real_type const & x,real_type const & y) const
      {
        real_type scale = static_cast<real_type>(1.0);
        real_type sum = 0;
        for(unsigned int octav = 0;octav<m_octaves;++octav)
        {
          real_type inv_scale = (1.0/scale);
          sum += std::fabs( m_noise(x*inv_scale,y*inv_scale)*scale );
          scale *= .5;
        }
        return sum;
      }

      real_type operator()(real_type const & x,real_type const & y,real_type const & z) const
      {
        real_type scale = static_cast<real_type>(1.0);
        real_type sum = 0;
        for(unsigned int octav = 0;octav<m_octaves;++octav)
        {
          real_type inv_scale = (1.0/scale);
          sum += std::fabs( m_noise(x*inv_scale,y*inv_scale,z*inv_scale)*scale );
          scale *= .5;
        }
        return sum;
      }

    };

  } // namespace noise

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_NOISE_NOISE_TURBULENCE_H
#endif
