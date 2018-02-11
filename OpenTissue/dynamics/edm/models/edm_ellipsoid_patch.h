#ifndef OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_ELLIPSOID_PATCH_H
#define OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_ELLIPSOID_PATCH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/edm/edm_surface.h>
#include <OpenTissue/core/math/math_constants.h>

#include <cmath>


namespace OpenTissue
{
  namespace edm
  {

    template<typename edm_types>
    class EllipsoidPatch
      : public Surface<edm_types>
    {
    public:

      typedef          Surface<edm_types>    base_type;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;

    public:

      EllipsoidPatch()
        : base_type()
      {
        base_type::set(NUM_NODES);
      }

      virtual ~EllipsoidPatch() {}

    private:

      vector3_type position(vector3_type const * a, real_type const & u, real_type const & v) const
      {
        real_type const u_ = u - u / this->m_M;
        // TODO: Comparing floats with == or != is not safe
        real_type const v_ = v == 0 ? 0.001 : v >= 1. ? 0.999 : v;
        real_type const theta = 2. * math::detail::pi<real_type>() * ( 1. - u_ );
        real_type const phi = math::detail::pi<real_type>() * v_;
        // a[0] contains the ellipsoid parameters
        vector3_type local = p( a[ 0 ][ 0 ], a[ 0 ][ 1 ], a[ 0 ][ 2 ], theta, phi );
        // a[1] contains the ellipsoid shape centre in WC
        return vector3_type(local + a[ 1 ]);
      }

      vector3_type normal(size_t m, size_t n) const
      {
        // I don't care about the magnitude of those estimated tangents,
        // as the final result will be normalized anyway!
        
        long lm = m;
        long ln = n;
        
        
        vector3_type const v = this->r( lm + 1, n ) - this->r( lm - 1, n );
        vector3_type const w = this->r( lm, n + 1 ) - this->r( lm, n - 1 );
        return vector3_type(
                            unit( cross(v , w) )
                           );
      }

    private:

      vector3_type p(real_type const & a, real_type const & b, real_type const & c, real_type const & theta, real_type const & phi) const
      {
        using std::cos;
        using std::sin;
        const real_type x = a * cos( theta ) * sin( phi );
        const real_type y = b * sin( theta ) * sin( phi );
        const real_type z = c * cos( phi );
        return vector3_type( x, y, z );
      }

    private:

      enum {NUM_NODES = 2};  ///< one for each natural and initial radius

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_ELLIPSOID_PATCH_H
#endif
