#ifndef OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_POLY6_H
#define OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_POLY6_H
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
    * W_poly6 Smoothing Kernel.
    * - This kernel has the gaussian bell property, but is much more efficient,
    *   e.g. no exp() of sqrt() required.
    * - This kernel should be used as default.
    */
    template< typename Types,
    struct OpenTissue::utility::RuntimeType<typename Types::real_type>* Radius,
      bool CheckRange >
    class WPoly6 : public FixedSmoothingKernel<Types, CheckRange>
    {
    public:
      typedef FixedSmoothingKernel<Types, CheckRange>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
    public:
      /**
      * Default Constructor (ohh, really?).
      */
      WPoly6() : base_type(*Radius)
      {
        m_k = 315./(64.*math::detail::pi<real_type>()*pow(base_type::m_radius, 9));
        m_l = -945./(32.*math::detail::pi<real_type>()*pow(base_type::m_radius, 9));
        m_m = -945./(32.*math::detail::pi<real_type>()*pow(base_type::m_radius, 9));
      }

    public:

      /**
      * W(r,h) = (315/(64 pi h^9))(h^2-|r|^2)^3
      *        = (315/(64 pi h^9))(h^2-r*r)^3
      */
      real_type evaluate(const vector& r) const
      {
        if (!FixedSmoothingKernel<Types, CheckRange>::checkRange(r))
          return 0.;
        register real_type res = base_type::m_radiusSqr-r*r;
        res *= res*res*m_k;
        return res;
      }

      /**
      * grad(W(r,h)) = r(-945/(32 pi h^9))(h^2-|r|^2)^2
      *              = r(-945/(32 pi h^9))(h^2-r*r)^2
      */
      vector gradient(const vector& r) const
      {
        if (!FixedSmoothingKernel<Types, CheckRange>::checkRange(r))
          return vector(0);
        register real_type tmp = base_type::m_radiusSqr-r*r;
        tmp *= tmp*m_l;
        return vector(tmp*r);
      }

      /**
      * laplacian(W(r,h)) = (-945/(32 pi h^9))(h^2-|r|^2)(-7|r|^2+3h^2)
      *                   = (-945/(32 pi h^9))(h^2-r*r)(3 h^2-7 r*r)
      */
      real_type laplacian(const vector& r) const
      {
        if (!FixedSmoothingKernel<Types, CheckRange>::checkRange(r))
          return 0.;
        const real_type tmp = r*r;
        register real_type res = (base_type::m_radiusSqr-tmp)*(3*base_type::m_radiusSqr-7*tmp);
        res *= m_m;
        return res;
      }

    protected:
      real_type  m_k;      ///< Normalization constant for the Kernel.
      real_type  m_l;      ///< Normalization constant for the Gradient.
      real_type  m_m;      ///< Normalization constant for the Laplacian.

    }; // End class WPoly6

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_POLY6_H
#endif
