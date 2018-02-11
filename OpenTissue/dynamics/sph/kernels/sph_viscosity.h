#ifndef OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_VISCOSITY_H
#define OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_VISCOSITY_H
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
    * W_viscosity Smoothing Kernel.
    * - This kernel is used for the viscosity force.
    * - It has been designed to handle both viscosity and normal damping.
    */
    template< typename Types,
    struct OpenTissue::utility::RuntimeType<typename Types::real_type>* Radius,
      bool CheckRange >
    class WViscosity : public FixedSmoothingKernel<Types, CheckRange>
    {
    public:
      typedef FixedSmoothingKernel<Types, CheckRange>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
    public:
      /**
      * Default Constructor.
      */
      WViscosity() : base_type(*Radius)
      {
        m_inv_hSqr = 1./base_type::m_radiusSqr;
        m_inv_2hTri = m_inv_hSqr/(2.*base_type::m_radius);
        m_k = (15./math::detail::pi<real_type>())*m_inv_2hTri;
        m_l = (15./math::detail::pi<real_type>())*m_inv_2hTri;
        m_m = 45./(math::detail::pi<real_type>()*pow(base_type::m_radius, 6));
      }

    public:

      /**
      * W(r,h) = (15/(2 pi h^3))(-|r|^3/(2 h^3)+|r|^2/h^2+h/(2 |r|)-1)
      */
      real_type evaluate(const vector& r) const
      {
        if (!checkRange(r))
          return 0.;
        register real_type tmp = length(r);
        if (tmp <= 0.)
          return 1e38;  // +oo
        tmp = -m_inv_2hTri*tmp*tmp*tmp + m_inv_hSqr*tmp*tmp + (base_type::m_radius/(2.*tmp)) - 1.;
        return real_type(m_k*tmp);
      }

      /**
      * grad(W(r,h)) = r(15/(2 pi h^3))(-(3|r|/2h^3) + 2/h^2 - h/2|r|^3)
      */
      vector gradient(const vector& r) const
      {
        if (!checkRange(r))
          return vector(0);
        register real_type tmp = length(r);
        if (tmp <= 0.) {
          vector rnd; 
          random(rnd,-0.0001, 0.0001);
          return vector(-1e38*rnd);  // -oo
        }
        tmp = -3*tmp*m_inv_2hTri + 2/base_type::m_radiusSqr - base_type::m_radiusSqr/(2*tmp*tmp*tmp);
        return vector((m_l*tmp)*r);
      }

      /**
      * laplacian(W(r,h)) = (45/(pi h^6))(h-|r|)
      */
      real_type laplacian(const vector& r) const
      {
        if (!checkRange(r))
          return 0.;
        return real_type(m_m*(base_type::m_radius-length(r)));
      }

    protected:
      real_type  m_k;      ///< Normalization constant for the Kernel.
      real_type  m_l;      ///< Normalization constant for the Gradient.
      real_type  m_m;      ///< Normalization constant for the Laplacian.
      real_type  m_inv_hSqr;   ///< Inverse radius squared.
      real_type  m_inv_2hTri;   ///< Radius triplet.

    }; // End class WViscosity

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_VISCOSITY_H
#endif
