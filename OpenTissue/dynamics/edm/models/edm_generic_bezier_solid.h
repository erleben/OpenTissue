#ifndef OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_GENERIC_BEZIER_SOLID_H
#define OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_GENERIC_BEZIER_SOLID_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/edm/edm_solid.h>
#include <OpenTissue/core/math/math_functions.h>
#include <vector>
#include <cmath>


namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types>
    class GenericBezierSolid
      : public Solid<edm_types>
    {
    public:

      typedef          Solid<edm_types>         base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;

    public:

      GenericBezierSolid()
        : base_type()
        , m_n(0)
      {}

      virtual ~GenericBezierSolid() {}

      void set(size_t order)
      {
        using std::pow;
        base_type::set(static_cast<size_t>(pow(static_cast<real_type>(order)+value_traits::one(),3)));
        m_n = order;
      }

    private:

      vector3_type position(vector3_type const * a, real_type const & u, real_type const & v, real_type const & w) const
      {
        std::vector<real_type> U(m_n+1), V(m_n+1), W(m_n+1);
        for (size_t i = 0; i <= m_n; ++i)
        {
          U[i] = B(i,u);
          V[i] = B(i,v);
          W[i] = B(i,w);
        }
        vector3_type Q;
        for (size_t i = 0; i <= m_n; ++i)
          for (size_t j = 0; j <= m_n; ++j)
            for (size_t k = 0; k <= m_n; ++k)
              Q += a[i*(m_n+1)*(m_n+1)+j*(m_n+1)+k]*W[i]*V[j]*U[k];
        return vector3_type(Q);
      }

      vector3_type normal(size_t l, size_t m, size_t n) const
      {
        // only calculate normals for particles on the surface of the volume
        if (!(n==0||m==0||l==0||n==this->m_N-1||m==this->m_M-1||l==this->m_L-1))
          return vector3_type(value_traits::zero());
        
        // 2009-11-03 kenny: Why do r only accept longs?
        long lm = m;
        long ln = n;
        long ll = l;
                
        vector3_type const u = this->r(ll+1,lm,ln) - this->r(ll-1,lm,ln);
        vector3_type const v = this->r(ll,lm+1,ln) - this->r(ll,lm-1,ln);
        vector3_type const w = this->r(ll,lm,ln+1) - this->r(ll,lm,ln-1);
        
        vector3_type normal_;
        if (n==0)
          normal_ += cross( u, v);
        else if (n==this->m_N-1)
          normal_ += cross( v, u);
        if (m==0)
          normal_ += cross( w, u);
        else if (m==this->m_M-1)
          normal_ += cross( u, w);
        if (l==0)
          normal_ += cross( v, w);
        else if (l==this->m_L-1)
          normal_ += cross( w, v);
        return vector3_type(unit(normal_));
      }

    private:

      real_type B(size_t i, const real_type& t) const
      {
        using std::pow;
        return real_type(pow(t,static_cast<int>(i))*pow(value_traits::one()-t,static_cast<int>(m_n-i))*math::fac<real_type>(m_n)/math::fac<real_type>(i)/math::fac<real_type>(m_n-i));
      }

    private:

      size_t  m_n;

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_GENERIC_BEZIER_SOLID_H
#endif
