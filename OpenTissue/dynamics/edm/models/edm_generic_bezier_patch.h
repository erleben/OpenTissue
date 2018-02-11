#ifndef OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_GENERIC_BEZIER_PATCH_H
#define OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_GENERIC_BEZIER_PATCH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/edm/edm_surface.h>
#include <OpenTissue/core/math/math_functions.h>

#include <vector>
#include <cmath>


namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types>
    class GenericBezierPatch
      : public Surface<edm_types>
    {
    public:

      typedef          Surface<edm_types>       base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;

    public:
      GenericBezierPatch() 
        : base_type()
        , m_n(0)
      {}

      virtual ~GenericBezierPatch() {}
      
      void set(size_t order)
      {
        using std::pow;
        base_type::set(static_cast<size_t>(pow(static_cast<real_type>(order)+value_traits::one(),2)));
        m_n = order;
      }

    private:

      vector3_type position(vector3_type const * a, real_type const & u, real_type const & v) const
      {
        std::vector<real_type> U(m_n+1), V(m_n+1);
        for (size_t i = 0; i <= m_n; ++i)
        {
          U[i] = B(i,u);
          V[i] = B(i,v);
        }
        vector3_type Q;
        for (size_t i = 0; i <= m_n; ++i)
          for (size_t j = 0; j <= m_n; ++j)
            Q += a[i*(m_n+1)+j]*V[i]*U[j];
        return vector3_type(Q);
      }

      vector3_type normal(size_t m, size_t n) const
      {
        // 2009-11-03 kenny: Why do r only accept longs?
        long lm = m;
        long ln = n;
                
        vector3_type const v = this->r(lm+1,ln) - this->r(lm-1,ln);
        vector3_type const w = this->r(lm,ln+1) - this->r(lm,ln-1);
        return vector3_type(unit( cross(v,w) ));
      }

    private:

      real_type B(size_t i, real_type const & t) const
      {
        using std::pow;
        return real_type(pow(t,static_cast<int>(i))*pow(value_traits::one()-t,static_cast<int>(m_n-i))*math::fac<real_type>(m_n)/math::fac<real_type>(i)/math::fac<real_type>(m_n-i));
      }

    private:

      size_t m_n;

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_GENERIC_BEZIER_PATCH_H
#endif
