#ifndef OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_ELLIPSOID_SOLID_H
#define OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_ELLIPSOID_SOLID_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/edm/edm_solid.h>

#include <OpenTissue/core/math/math_constants.h>

#include <cmath>


namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types>
    class EllipsoidSolid
      : public Solid<edm_types>
    {
    public:

      typedef          Solid<edm_types>          base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;

    public:

      EllipsoidSolid()
        : base_type()
      {
        base_type::set(NUM_NODES);
      }

      virtual ~EllipsoidSolid() {}

    private:

      vector3_type position(vector3_type const * a, real_type const & u, real_type const & v, real_type const & w) const
      {
        real_type const u_ = u-(u/this->m_M);
        // TODO: Comparing floats with == or != is not safe
        real_type const v_ = v==0?0.001:v>=1.?0.999:v;
        real_type const theta = value_traits::two()*math::detail::pi<real_type>()*(value_traits::one()-u_);
        real_type const phi = math::detail::pi<real_type>()*v_;
        vector3_type local = p(a[0][0], a[0][1], a[0][2], theta, phi, value_traits::one()-w);
        return vector3_type(local+a[1]);
      }

      vector3_type normal(size_t l, size_t m, size_t n) const
      {
        if (n>0)
          return vector3_type();
        // I don't care about the magnitude of those estimated tangents,
        // as the final result will be normalized anyway!
        
        // 2009-11-03 kenny: Why do r only accept longs?
        long lm = m;
        long ln = n;
        long ll = l;
        
        vector3_type const u = this->r(ll+1,lm,ln) - this->r(ll-1,lm,ln);
        vector3_type const v = this->r(ll,lm+1,ln) - this->r(ll,lm-1,ln);
        vector3_type const normal_ = u%v;
        return vector3_type(unit(normal_));
      }

    private:

      vector3_type p(real_type const & a, real_type const & b, real_type const & c, real_type const & theta, real_type const & phi, real_type const & rho) const
      {
        using std::cos;
        using std::sin;
        real_type const x = a*rho*cos(theta)*sin(phi);
        real_type const y = b*rho*sin(theta)*sin(phi);
        real_type const z = c*rho*cos(phi);
        return vector3_type(x,y,z);
      }

    private:

      enum {NUM_NODES = 2};  ///< one for each natural and initial radius

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_ELLIPSOID_SOLID_H
#endif
