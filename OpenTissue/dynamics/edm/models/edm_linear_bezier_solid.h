#ifndef OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_LINEAR_BEZIER_SOLID_H
#define OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_LINEAR_BEZIER_SOLID_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/edm/edm_solid.h>

#include <cmath>


namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types>
    class LinearBezierSolid
      : public Solid<edm_types>
    {
    public:

      typedef          Solid<edm_types>         base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;

    public:

      LinearBezierSolid() : base_type()
      {
        base_type::set(NUM_NODES);
      }

      virtual ~LinearBezierSolid() {}

    private:

      vector3_type position(vector3_type const * a, real_type const & u, real_type const & v, real_type const & w) const
      {
        real_type U[ORDER], V[ORDER], W[ORDER];
        for (size_t i=0; i < ORDER; ++i) {
          U[i] = B(i,u);
          V[i] = B(i,v);
          W[i] = B(i,w);
        }
        vector3_type Q;
        for (size_t i=0; i<ORDER; ++i)
          for(size_t j=0; j<ORDER; ++j)
            for(size_t k=0; k<ORDER; ++k)
              Q += a[i*ORDER*ORDER+j*ORDER+k]*U[i]*V[j]*W[k];
        return vector3_type(Q);
      }

      vector3_type normal(size_t l, size_t m, size_t n) const
      {
        // only calculate normals for particles on the surface of the volume
        if (!(n==0||m==0||l==0||n==this->m_N-1||m==this->m_M-1||l==this->m_L-1))
          return vector3_type(value_traits::zero());  // TODO use value traits for constants
        
        // 2009-11-03 kenny: Why do r only accept longs?
        long ln = n;
        long lm = m;
        long ll = l;
        
        vector3_type const u = this->r(ll+1, lm,   ln  ) - this->r(ll-1, lm,   ln  );
        vector3_type const v = this->r(ll,   lm+1, ln  ) - this->r(ll,   lm-1, ln  );
        vector3_type const w = this->r(ll,   lm,   ln+1) - this->r(ll,   lm,   ln-1);
        
        vector3_type normal_;
        if (n==0)
          normal_ += cross( u, v);
        else if (n==this->m_N-1)
          normal_ += cross( v, u);
        if (m==0)
          normal_ += cross( w, u );
        else if (m==this->m_M-1)
          normal_ += cross( u, w);
        if (l==0)
          normal_ += cross( v, w);
        else if (l==this->m_L-1)
          normal_ += cross( w, v);
        return vector3_type(unit(normal_));
      }

    private:

      real_type B(size_t n, real_type const & t) const
      {
        assert(n<ORDER);
        switch (n) {
          case 0: return real_type(value_traits::one()-t);  // B0(t)=1-t
          case 1: return real_type(t);     // B1(t)=t
        }
        return real_type(value_traits::zero());
      }

    private:

      enum {ORDER = 2, NUM_NODES = 8};  ///< ORDER actually is the order+1

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_LINEAR_BEZIER_SOLID_H
#endif
