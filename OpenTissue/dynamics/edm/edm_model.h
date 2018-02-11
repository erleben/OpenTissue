#ifndef OPENTISSUE_DYNAMICS_EDM_EDM_MODEL_H
#define OPENTISSUE_DYNAMICS_EDM_EDM_MODEL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_conjugate_gradient.h>

#include <cmath>
#include <list>
#include <map>
#include <vector>


namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types>
    class Model
      : public edm_types::model_traits
    {
    public:

      typedef typename edm_types::value_traits    value_traits;
      typedef typename edm_types::real_type       real_type;
      typedef typename edm_types::vector3_type    vector3_type;
      typedef typename edm_types::model_id_type       model_id_type;
      typedef typename edm_types::Particle        particle_type;
      typedef typename edm_types::force_type      force_type;
      typedef typename edm_types::object_type     object_type;

      typedef std::list<force_type const *>                Forces;
      typedef std::list<object_type const *>               Objects;
      typedef std::map<particle_type const *, Forces>      EDMLocalForces;
      typedef typename ublas::compressed_matrix<real_type> EDMMatrix;
      typedef typename ublas::vector<real_type>            EDMVector;
      
    private:
      
      virtual void compute_stiffness(EDMMatrix & K) const = 0;
      virtual void compute_surface_normals() = 0;
      virtual size_t particle_count() const = 0;
      virtual particle_type const & get_particle(size_t idx) const = 0;
      virtual particle_type & get_particle(size_t idx) = 0;
      
    protected:
      
      // particles are placed in their respective class types
      Forces  m_Fs;       ///< external forces acting on all particles
      EDMLocalForces  m_LFs; ///< specified external forces acting on a single particle
      Objects  m_Os;      ///< IIIOs the body should perform collision against
      real_type  m_dt;          ///< timestep (delta t)
      real_type  m_strength;
      vector3_type *  m_rest;
      vector3_type *  m_init;
      size_t  m_max_nodes;
      
    private:
      
      model_id_type  m_type;  // only for safe casting!
      
    public:

      Model(model_id_type type)
        : m_dt(value_traits::zero())
        , m_strength(value_traits::one())
        , m_rest(0)
        , m_init(0)
        , m_max_nodes(0)
        , m_type(type)
      {
      }

      virtual ~Model()
      {
        delete[] m_rest;
        delete[] m_init;
      }

      model_id_type const & type() const  // TODO: we really should use interfaces!
      {
        return m_type;
      }

      real_type & strength()
      {
        return m_strength;
      }

      real_type const & strength() const
      {
        return m_strength;
      }

      real_type & timestep()
      {
        return m_dt;
      }

      real_type const & timestep() const
      {
        return m_dt;
      }

    public:

      Model & add(force_type const & f)
      {
        remove(f);
        m_Fs.push_back(&f);
        return *this;
      }

      Model & remove(force_type const & f)
      {
        m_Fs.remove(&f);
        return *this;
      }

      Model & add(particle_type const & a, force_type const & f)
      {
        remove(a, f);
        Forces& F = m_LFs[&a];
        F.push_back(&f);
        return *this;
      }

      Model & remove(particle_type const & a, force_type const & f)
      {
        Forces& F = m_LFs[&a];
        F.remove(&f);
        return *this;
      }

      Model & add(object_type const & o)
      {
        remove(o);
        m_Os.push_back(&o);
        return *this;
      }

      Model & remove(object_type const & o)
      {
        m_Os.remove(&o);
        return *this;
      }

    public:

      size_t num_particles() const
      {
        return size_t(particle_count());
      }

      vector3_type const & position(size_t idx) const
      {
        return get_particle(idx).r;
      }

      particle_type const & particle(size_t idx) const
      {
        return get_particle(idx);
      }

      bool lock_particle(size_t idx)
      {
        if (idx >= particle_count())
          return false;
        get_particle(idx).f = true;
        return true;
      }

      bool unlock_particle(size_t idx)
      {
        if (idx >= particle_count())
          return false;
        get_particle(idx).f = false;
        return true;
      }

      bool move_particle(size_t idx, vector3_type const & displacement)
      {
        if (idx >= particle_count())
          return false;
        get_particle(idx).r += displacement;
        return true;
      }

    protected:

      vector3_type compute_external_forces(particle_type const & a) const
      {
        vector3_type f(value_traits::zero());
        typename Forces::const_iterator F, endF = m_Fs.end();
        // first all external forces
        for (F = m_Fs.begin(); F != endF; ++F)
          f += (*F)->apply(a);
        // then all local forces
        typename EDMLocalForces::const_iterator LF = m_LFs.find(&a);
        if (LF != m_LFs.end())
        {
          endF = LF->second.end();
          for (F = LF->second.begin(); F != endF; ++F)
            f += (*F)->apply(a);
        }
        return vector3_type(f);
      }

      bool collision_projection(vector3_type & r) const
      {
        bool collision = false;
        typename Objects::const_iterator O, end = m_Os.end();
        for (O = m_Os.begin(); O != end; ++O)
        {
          real_type const penetration = (*O)->eval(r);
          if (penetration >= 0)
            continue;
          vector3_type const n = (*O)->normal(r);
          r += (*O)->dist(r)*n;
          collision = true;
        }
        return collision;
      }

      void set(size_t num_nodes)
      {
        delete[] m_rest;
        delete[] m_init;
        m_rest = new vector3_type[num_nodes];
        m_init = new vector3_type[num_nodes];
        m_max_nodes = num_nodes;
      }

    public:

      void run(bool compute_elasticity)
      {
        size_t const size = particle_count();
        EDMMatrix K(size, size);
        K.clear();
        compute_stiffness(K);

        if (compute_elasticity)
        {
          EDMVector rX(size), rY(size), rZ(size);
          rX.clear(); rY.clear(); rZ.clear();
          for (size_t i = 0; i < size; ++i)
          {
            particle_type & a = get_particle(i);
            rX(i) = a.r(0);
            rY(i) = a.r(1);
            rZ(i) = a.r(2);
          }
          EDMVector eX(size), eY(size), eZ(size);
          eX.clear(); eY.clear(); eZ.clear();
          ublas::axpy_prod( K, rX, eX, true );
          ublas::axpy_prod( K, rY, eY, true );
          ublas::axpy_prod( K, rZ, eZ, true );
          for (size_t i = 0; i < size; ++i)
            get_particle(i).E = vector3_type(eX(i), eY(i), eZ(i));
        }

        EDMVector gX(size), gY(size), gZ(size);
        gX.clear(); gY.clear(); gZ.clear();
        for (size_t i = 0; i < size; ++i)
        {
          particle_type & a = get_particle(i);
          real_type const mc1 = (1./(m_dt*m_dt))*a.m + (.5*(1./m_dt))*a.g;
          real_type const mc2 = a.m/m_dt - .5*a.g;
          a.F = compute_external_forces(a);  // calculate all the external forces acting on a!
          K(i, i) += mc1;
          gX(i) = a.F(0) + mc1*a.r(0) + mc2*a.v(0);
          gY(i) = a.F(1) + mc1*a.r(1) + mc2*a.v(1);
          gZ(i) = a.F(2) + mc1*a.r(2) + mc2*a.v(2);
        }

        EDMVector rX(size), rY(size), rZ(size);
        rX.clear(); rY.clear(); rZ.clear();

        static real_type const EDM_CG_TOLERANCE  = 0.00001;

        real_type const E = EDM_CG_TOLERANCE*size;

        size_t iterations; ///< This variable holds the number of used iterations in the conjugate gradient solver

        math::big::conjugate_gradient(K, rX, gX, size, E*E, iterations);
        math::big::conjugate_gradient(K, rY, gY, size, E*E, iterations);
        math::big::conjugate_gradient(K, rZ, gZ, size, E*E, iterations);

        // update all positions, velocities, etc.
        for (size_t i = 0; i < size; ++i)
        {
          particle_type & a = get_particle(i);
          a.o = a.r;
          if (!a.f)  // only update pos if particle isn't fixed/locked
            a.r = vector3_type(rX(i), rY(i), rZ(i));
          vector3_type new_r = a.r;
          if (collision_projection(new_r))
            a.r = new_r;
          a.v = (a.r - a.o)/m_dt;
        }

        compute_surface_normals();
      }

    };


   /**
    * Zeroize utility.
    *
    * @param val      Value to be zeroized.
    * @param epsilon  Threshold value for numerical instabilities while assembling the stiffness matrix.
    */
    // 2009-03-11 kenny: argh, very ugly static cast, why not create an overloaded 2 argument version then use boost::numeric_cast to initialize a local constant used for calling the three-argument version? 
    template<typename real_type>
    inline real_type zeroize(real_type const & val, real_type const & epsilon = static_cast<real_type>(1./8192))
    {
        using std::fabs;
        return real_type(fabs(val) < epsilon ? math::ValueTraits<real_type>::zero() : val);
    }

  }  // namespace edm

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_EDM_MODEL_H
#endif
