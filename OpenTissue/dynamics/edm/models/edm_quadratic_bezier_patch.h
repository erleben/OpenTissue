#ifndef OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_QUADRATIC_BEZIER_PATCH_H
#define OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_QUADRATIC_BEZIER_PATCH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/edm/edm_surface.h>

#include <cmath>
#include <cassert>


namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types>
    class QuadraticBezierPatch
      : public Surface<edm_types>
    {
    public:

      typedef          Surface<edm_types>       base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;

     public:

      QuadraticBezierPatch()
        : base_type()
      {
        base_type::set(NUM_NODES);
      }

      virtual ~QuadraticBezierPatch() {}

    private:

      vector3_type position(vector3_type const * a, real_type const & u, real_type const & v) const
      {
        real_type U[ORDER], V[ORDER];
        for (size_t i = 0; i < ORDER; ++i) {
          U[i] = B(i,u);
          V[i] = B(i,v);
        }
        vector3_type Q;
        for (size_t i = 0; i < ORDER; ++i)
          for (size_t j = 0; j < ORDER; ++j)
            Q += a[i*ORDER+j]*U[j]*V[i];
        return vector3_type(Q);
      }

      vector3_type normal(size_t m, size_t n) const
      {
        // I don't care about the magnitude of those estimated tangents,
        // as the final result will be normalized anyway!
        
        
        // 2009-11-03 kenny: Why do r only accept longs?
        long lm = m;
        long ln = n;
        
        vector3_type const v = this->r(lm+1,ln  ) - this->r(lm-1,ln  );
        vector3_type const w = this->r(lm,  ln+1) - this->r(lm,  ln-1);
        return vector3_type(unit( cross(v,w) ));
      }

      size_t max_nodes() const
      {
        return NUM_NODES;
      }

    private:

      real_type B(size_t n, real_type const & t) const
      {
        assert(n<ORDER);
        real_type const tmp = value_traits::one()-t;
        switch (n) {
          case 0: return real_type(tmp*tmp);   // B0(t)=(1-t)^2
          case 1: return real_type(value_traits::two()*t*tmp);  // B1(t)=2t(1-t)
          case 2: return real_type(t*t);       // B2(t)=t^2
        }
        return real_type(value_traits::zero());
      }

      real_type dBdt(size_t n, real_type const & t) const
      {
        assert(n<ORDER);
        const real_type tmp = value_traits::two()*t;
        switch (n) {
          case 0: return real_type(tmp-value_traits::two());      // B0'(t)=(t^2-2t+1)'=2t-2
          case 1: return real_type(-value_traits::two()*tmp+value_traits::two());  // B1'(t)=(-2t^2+2t)'=-4t+2
          case 2: return real_type(tmp);         // B2'(t)=(t^2)'     =2t
        }
        return real_type(value_traits::zero());
      }

      real_type ddtdBdt(size_t n, real_type const & t) const
      {
        assert(n<ORDER);
        switch (n) {
          case 0: return real_type(value_traits::two());   // B0''(t)=(t^2-2t+1)''=(2t-2)' =2
          case 1: return real_type(-value_traits::four());  // B1''(t)=(-2t^2+2t)''=(-4t+2)'=-4
          case 2: return real_type(value_traits::two());   // B2''(t)=(t^2)''     =(2t)'   =2
        }
        return real_type(t);
      }

    private:

      enum {ORDER = 3, NUM_NODES = 9};  ///< actually it's ORDER+1

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_MODELS_EDM_QUADRATIC_BEZIER_PATCH_H
#endif
