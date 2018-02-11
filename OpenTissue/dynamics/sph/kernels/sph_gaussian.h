#ifndef OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_GAUSSIAN_H
#define OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_GAUSSIAN_H
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
    *                        1           |r|^2
    * Gaussian : W(r,h) = -------- exp(- -----)
    * <Monaghan 1992>     h pi^0.5        h^2
    *
    *                                      1             |r|^2
    * Isotropic Gaussian : W(r,h) = -------------- exp(- -----)
    * <Erleben, et al. 2005>        (2 pi h^2)^1.5       2 h^2
    *
    */

    /**
    * W_gaussian Smoothing Kernel (fixed kernel support).
    */
    template< typename Types,
    struct OpenTissue::utility::RuntimeType<typename Types::real_type>* Radius,
      bool CheckRange >
    class WFixedGaussian : public FixedSmoothingKernel<Types, CheckRange>
    {
    public:
      typedef FixedSmoothingKernel<Types, CheckRange>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
    public:
      /**
      * Default Constructor.
      */
      WFixedGaussian() : base_type(*Radius)
      {
        m_inv_2hSqr = 1./(2.*this->m_radiusSqr);
        m_3hSqr = 3.*this->m_radiusSqr;

        m_k = 1./pow(2*math::detail::pi<real_type>()*this->m_radiusSqr, 1.5);
        m_l = sqrt(2.)/(4.*math::detail::pi<real_type>()*this->m_radiusSqr*this->m_radiusSqr*sqrt(math::detail::pi<real_type>()*this->m_radiusSqr));
        m_m = sqrt(2.)/(4.*math::detail::pi<real_type>()*this->m_radiusSqr*this->m_radiusSqr*this->m_radiusSqr*sqrt(math::detail::pi<real_type>()*this->m_radiusSqr));
      }

      /**
      * Deconstructor.
      */
      ~WFixedGaussian()
      {
      }

    public:
      /**
      * W(r,h) = (1/(2 pi h^2)^1.5)exp(-r*r/(2 h^2))
      */
      real_type evaluate(const vector& r) const
      {
        real_type res = 0;
        if (!checkRange(r))
          return res;
        res = (r*r)*m_inv_2hSqr;
        res = m_k*::exp(-res);
        return res;
      }

      /**
      * grad(W(r,h)) = r(sqrt(2)/(4 pi h^4 sqrt(pi h^2)))exp(-r*r/(2 h^2))
      */
      vector gradient(const vector& r) const
      {
        vector res;
        res *= 0;
        if (!checkRange(r))
          return res;
        real_type tmp = (r*r)*m_inv_2hSqr;
        tmp = m_l*::exp(-tmp);
        res = tmp*r;
        return res;
      }

      /**
      * laplacian(W(r,h)) = (sqrt(2)/(4 pi h^6 sqrt(pi h^2)))exp(-r*r/(2 h^2))(r*r-3h^2)
      */
      real_type laplacian(const vector& r) const
      {
        real_type res = 0;
        if (!checkRange(r))
          return res;
        const real_type tmp = r*r;
        res = tmp*m_inv_2hSqr;
        res = m_m*::exp(-res);
        res *= tmp-m_3hSqr;
        return res;
      }
    protected:
      real_type  m_k;      ///< Magic Gaussian constant for eval.
      real_type  m_l;      ///< Magic Gaussian constant for gradient.
      real_type  m_m;      ///< Magic Gaussian constant for laplacian.
      real_type  m_inv_2hSqr;   ///< Inverse 2 radius squared.
      real_type  m_3hSqr;   ///< 3 radius squared.

    }; // End class WFixedGaussian

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_KERNELS_SPH_GAUSSIAN_H
#endif
