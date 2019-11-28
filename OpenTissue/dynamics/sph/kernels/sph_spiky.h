#ifndef OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_SPIKY_H
#define OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_SPIKY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_kernel.h>
#include <OpenTissue/core/math/math_constants.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * W_spiky Smoothing Kernel.
    * - This kernel is used for the pressure force, as the gaussian gradient will introduce clustering.
    */
    template< typename Types,
    struct OpenTissue::utility::RuntimeType<typename Types::real_type>* Radius,
      bool CheckRange >
    class WSpiky : public FixedSmoothingKernel<Types, CheckRange>
    {
    public:
      typedef FixedSmoothingKernel<Types, CheckRange>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
    public:
      /**
      * Default Constructor.
      */
      WSpiky() : base_type(*Radius)
      {
        m_k = 15./(math::detail::pi<real_type>()*pow(base_type::m_radius, 6));
        m_l = -45./(math::detail::pi<real_type>()*pow(base_type::m_radius, 6));
        m_m = -90./(math::detail::pi<real_type>()*pow(base_type::m_radius, 6));
      }

    public:

      /**
      * W(r,h) = (15/(pi h^6))(h-|r|)^3
      */
      real_type evaluate(const vector& r) const
      {
        if (!FixedSmoothingKernel<Types, CheckRange>::checkRange(r))
          return 0.;
        register real_type res = base_type::m_radius- length(r);
        res *= res*res*m_k;
        return res;
      }

      /**
      * grad(W(r,h)) = r(-45/(pi h^6))((h-|r|)^2)/|r|
      */
      vector gradient(const vector& r) const
      {
        if (!FixedSmoothingKernel<Types, CheckRange>::checkRange(r))
          return vector(0);
        register real_type tmp = length(r);
        if (tmp <= 0.) {
          vector rnd; 
          random(rnd,-0.0001, 0.0001);
          return vector(m_l*rnd);
        }
        const register real_type tmp2 = base_type::m_radius-tmp;
        tmp = (tmp2*tmp2)/tmp;
        return vector((m_l*tmp)*r);
      }

      /**
      * laplacian(W(r,h)) = (-90/(pi h^6))((h-|r|)(h-2|r|))/|r|
      */
      real_type laplacian(const vector& r) const
      {
        if (!FixedSmoothingKernel<Types, CheckRange>::checkRange(r))
          return 0.;
        register real_type tmp = length(r);
        if (tmp <= 0.)
          return -1e38; // -oo
        tmp = ((base_type::m_radius-tmp)*(base_type::m_radius-2*tmp))/tmp;
        return real_type(m_m*tmp);
      }

    protected:
      real_type  m_k;      ///< Normalization constant for the Kernel.
      real_type  m_l;      ///< Normalization constant for the Gradient.
      real_type  m_m;      ///< Normalization constant for the Laplacian.

    }; // End class WSpiky

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_SPIKY_H
#endif
