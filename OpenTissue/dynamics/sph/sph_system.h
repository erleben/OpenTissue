#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_SYSTEM_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_material.h>
#include <vector>

namespace OpenTissue
{
  namespace sph
  {

    template< typename Types >
    class FluidHashPolicy
    {
    public:
      typedef typename Types::real_type  real_type;
      typedef typename Types::particle  data_type;
      typedef typename Types::particle  query_type;
      typedef typename Types::particle_cptr_pair  result_type;
#if defined(SPHSH_PARALLEL)
      typedef typename Types::particle_cptr_pair_container  result_container;
#else
      typedef typename Types::particle_cptr_container  result_container;
#endif
      typedef typename Types::vector  vector3_type;

    public:
      void reset(result_container& results) const
      {
        results.clear();
      }

      void report(const data_type& data, const query_type& query, result_container& results) const
      {
        if ((!data.fixed() || &data == &query) && data.check(query.position()))
#if defined(SPHSH_PARALLEL)
          if (&query != &data)
            results.push_back(result_type(&query,&data));
#else
          results.push_back(&data);
#endif
      }

      vector3_type min_coord(const typename Types::particle& p) const {return p.min();}
      vector3_type max_coord(const typename Types::particle& p) const {return p.max();}
      const vector3_type& position(const typename Types::particle& p) const {return p.position();}
    };


    /**
    * SPH System Class.
    */
    template< typename Types,
      typename DensitySolver,
      typename PressureSolver,
      typename NormalSolver,
      typename GravityForce,
      typename BuoyancyForce,
      typename PressureForce,
      typename ViscosityForce,
      typename SurfaceForce,
      typename IntegratorPolicy,
      typename ColorField >
    class System
    {
    public:
      typedef typename Types::real_type                     real_type;
      typedef typename Types::vector                        vector;
      typedef typename Types::particle                      particle;
      typedef typename Types::particle_container            particle_container;
      typedef typename Types::particle_cptr_container       particle_cptr_container;
      typedef typename Types::particle_cptr_pair_container  particle_cptr_pair_container;
      typedef typename Types::collision_detection           collision_detection;

      typedef          Material<Types>                   fluid_material;

      typedef          FluidHashPolicy<Types>            fluid_hash_policy;
      typedef typename Types::template hashing<fluid_hash_policy, typename fluid_hash_policy::data_type>  fluid_hashing;
      typedef typename fluid_hashing::hash_grid             fluid_hash_grid;
      typedef typename fluid_hashing::point_query           fluid_point_query;

    public:
      /**
      * Default Constructor.
      */
      System()
        : m_integrator(NULL)
        , m_density(NULL)
        , m_pressure(NULL)
        , m_normal(NULL)
        , m_gravityForce(NULL)
        , m_buoyancyForce(NULL)
        , m_pressureForce(NULL)
        , m_viscosityForce(NULL)
        , m_surfaceForce(NULL)
        , m_color(NULL)
        , m_material(NULL)
      {}

      /**
      * Deconstructor.
      */
      virtual ~System()
      {
        cleanUp();
      }

    public:
      /**
      * Create the SPH system.
      *
      * @param particle_mass  sets the const mass for all particles in this system
      */
      template<typename MaterialPolicy>
      bool create(const MaterialPolicy& material, const vector& gravity)
      {
        // clean up previously used stuff
        cleanUp();

        m_material = &material;
        m_integrator = new IntegratorPolicy(material.timestep(), material.restitution());
        m_density = new DensitySolver;
        m_pressure = new PressureSolver(material.gas_stiffness(), material.density());
        m_normal = new NormalSolver;
        m_gravityForce = new GravityForce(gravity);
        m_buoyancyForce = new BuoyancyForce(material.buoyancy(), material.density(), gravity);
        m_pressureForce = new PressureForce;
        m_viscosityForce = new ViscosityForce(material.viscosity());
        m_surfaceForce = new SurfaceForce(material.tension(), material.threshold());
        m_color = new ColorField;

        return true;
      }

      /**
      * Hash Table Initialization.
      */
      bool initHashing(size_t size, const real_type& spacing)
      {
        if (!m_particles.empty())  // system init must be called afterwards
          return false;
        m_search.resize(size);
        m_search.set_spacing(spacing);
        return true;
      }

      collision_detection& collisionSystem()
      {
        return m_colisys;
      }

      const collision_detection& collisionSystem() const
      {
        return m_colisys;
      }

      /**
      * System Initialization of particles (positions)
      */
      template< typename PositionIterator >
      bool init(const PositionIterator& begin, const PositionIterator& end)
      {
        return init<PositionIterator, PositionIterator>(begin, end, end, end);
      }

      /**
      * System Initialization of particles (positions and velocities)
      */
      template< typename PositionIterator, typename VelocityIterator >
      bool init(const PositionIterator& pbegin, const PositionIterator& pend, const VelocityIterator& vbegin, const VelocityIterator& vend)
      {
        VelocityIterator vel = vbegin;
        for (PositionIterator pos = pbegin; pos != pend; ++pos) {
          particle par;
          par.mass() = m_material->particle_mass();
          par.velocity().clear();
          par.normal().clear();
          par.position_old() = par.position() = *pos;

          if (vel != vend) {
            par.velocity() = *vel;
            par.position_old() -= *vel;
            ++vel;
          }

          m_particles.push_back(par);
        }

        // init const pointer (read only) particle constainer
        m_cptr_particles.resize(m_particles.size());
        for (typename particle_container::const_iterator par = m_particles.begin(); par != m_particles.end(); ++par)
          m_cptr_particles.push_back(&*par);

#if defined(SPHSH)
        // update hash grid
        m_search.init_data(m_particles.begin(), m_particles.end());
#endif
        // init dynamics (no integration)
        if (!solve())
          return false;

        // init integrator (no integration)
        m_integrator->setCollisionSystem(&m_colisys);
        m_integrator->initialize_particles(m_particles.begin(), m_particles.end());

        return true;
      }


      /**
      * System Initialization of particles using emitter.
      */
      template< typename EmitterPolicy >
      bool init(EmitterPolicy& emitter, size_t particles)
      {
        particle par;
        par.mass() = m_material->particle_mass();
        par.velocity().clear();
        par.normal().clear();
        par.position_old().clear();
        par.position().clear();

        for (size_t n = 0; n < particles; ++n)
          m_particles.push_back(par);

        // fix all particles and set pos + vel (the emitter controls the release)
        if (!emitter.initialize(m_particles.begin()+m_particles.size()-particles, m_particles.end()))
          return false;

        // init const pointer (read only) particle constainer
        m_cptr_particles.resize(m_particles.size());
        for (typename particle_container::const_iterator par = m_particles.begin(); par != m_particles.end(); ++par)
          m_cptr_particles.push_back(&*par);

#if defined(SPHSH)
        // update hash grid
        m_search.init_data(m_particles.begin(), m_particles.end());
#endif
        // init dynamics (no integration)
        if (!solve())
          return false;

        // init integrator (no integration)
        m_integrator->setCollisionSystem(&m_colisys);
        m_integrator->initialize_particles(m_particles.begin(), m_particles.end());

        return true;
      }

      /**
      * Retrieve particles.
      */
      const particle_container &particles() const
      {
        return m_particles;
      }

      /**
      * Retrieve material.
      */
      const fluid_material* material() const
      {
        return m_material;
      }

      /**
      * Calculate iso value from pos.
      */
      typename ColorField::value isoValue(const vector& pos)
      {
        static particle temp__par;
        temp__par.position() = pos;
        particle_cptr_container &particles = m_cptr_particles;
#if defined(SPHSH) && !defined(SPHSH_PARALLEL)
        // TODO: Currently the following line does not compile using SPHSH_PARALLEL
        m_search(temp__par, particles, typename fluid_point_query::all_tag());
#endif
        return m_color->apply(temp__par, particles.begin(), particles.end());
      }

#if defined(SPHSH) && defined(SPHSH_PARALLEL)

      /**
      * Parallel Solver.
      */
      bool solve()
      {
        if (m_particles.empty()) return false;

        particle_cptr_pair_container particles;
        m_search(m_particles.begin(), m_particles.end(), particles, typename fluid_point_query::all_tag() );
        typename particle_container::iterator par_end = m_particles.end();
        typename particle_cptr_pair_container::const_iterator pair_end = particles.end();

        // reset particles, calc own densities and external forces
        for (typename particle_container::iterator p = m_particles.begin(); p != par_end; ++p) {
          particle &par = *p;
          par.density() = m_density->apply(par, par);
          if (m_material->buoyancy() > 0)
            par.force() = m_buoyancyForce->apply(par, par);
          else
            par.force() = m_gravityForce->apply(par, par);
          par.normal() = m_normal->apply(par, par);
        }

        // calc densities
        for (typename particle_cptr_pair_container::const_iterator pp = particles.begin(); pp != pair_end; ++pp) {
          const particle &p_i = *pp->first;
          const particle &p_j = *pp->second;
          particle &par = const_cast<particle&>(p_i);
          par.density() += m_density->apply(p_i, p_j);
        }

        // calc pressures
        for (typename particle_container::iterator p = m_particles.begin(); p != par_end; ++p) {
          particle &par = *p;
          par.pressure() = m_pressure->apply(par, par);
        }

        // calc normals and sph forces
        for (typename particle_cptr_pair_container::const_iterator pp = particles.begin(); pp != pair_end; ++pp) {
          const particle &p_i = *pp->first;
          const particle &p_j = *pp->second;
          particle &par = const_cast<particle&>(p_i);
          vector& f = par.force();

          // calc normal
          par.normal() += m_normal->apply(p_i, p_j);

          // pressure force contribution
          f += m_pressureForce->apply(p_i, p_j);

          // viscosity force contribution
          f += m_viscosityForce->apply(p_i, p_j);

          // surface tension force contribution
          // - currently not supported in the parallel solver :(
          //f += m_surfaceForce->apply(p_i, p_j);
        }

        return true;
      }

#else

      /**
      * Sequential Solver.
      */
      bool solve()
      {
        if (m_particles.empty()) return false;

        particle_cptr_container &particles = m_cptr_particles;

        //#if defined(SPHSH)
        //      particle_cptr_container particles;
        //#else
        //      particle_cptr_container &particles = m_cptr_particles;
        //#endif

        typename particle_container::iterator end = m_particles.end();

        //      int pars = 0;
        //      double dens = 0.;
        //      int fixed = 0;
        for (typename particle_container::iterator p = m_particles.begin(); p != end; ++p) {
          particle &par = *p;

#if defined(SPHSH)
          m_search(par, particles, typename fluid_point_query::all_tag() );
#endif
          /*
          vector disp = 0;
          typename particle_cptr_container::const_iterator chkend = particles.end();
          for (typename particle_cptr_container::const_iterator chk = particles.begin(); chk != chkend; ++chk)
          if (*chk != &par && sqr_length((*chk)->position()-par.position()) <= 0.)
          {
          static int c = 0;
          const real_type s = sgn(par.velocity()(c));
          disp(c) += 0.0001*(s?s:1);
          c = (c+1)%3;
          fixed++;
          break;
          }
          par.position() += disp;
          */

          typename particle_cptr_container::const_iterator begin = particles.begin();
          typename particle_cptr_container::const_iterator end = particles.end();

          // calc density
          //m_density->apply(par, m_cptr_particles);
          par.density() = m_density->apply(par, begin, end);

          // calc pressure
          par.pressure() = m_pressure->apply(par, begin, end);

          //        pars += particles.size();
          //        dens += par.density();
        }
        //      if (fixed) cout << "Particles Fixed: " << fixed << endl;
        //      cout << "AVG Particles: " << pars/m_particles.size() << endl;
        //      cout << "AVG Density: " << dens/m_particles.size() << endl;

        //double mnorm = 1000000., norm = 0., Mnorm = -1.;
        for (typename particle_container::iterator p = m_particles.begin(); p != end; ++p) {
          particle &par = *p;

#if defined(SPHSH)
          m_search(par, particles, typename fluid_point_query::all_tag());
#endif

          typename particle_cptr_container::const_iterator begin = particles.begin();
          typename particle_cptr_container::const_iterator end = particles.end();

          // calc surface normal
          par.normal() = m_normal->apply(par, begin, end);
          //const double nm = length(par.normal());
          //norm += nm;
          //Mnorm = std::max(Mnorm, nm);
          //mnorm = std::min(mnorm, nm);

          // calc forces
          vector& f = par.force();
          f.clear();  // reset forces

          // calc pressure force
          f += m_pressureForce->apply(par, begin, end);

          // calc viscosity force
          f += m_viscosityForce->apply(par, begin, end);

          // calc surface tension force
          if (m_material->tension() > 0)
            f += m_surfaceForce->apply(par, begin, end);

          // external forces

          // calc gravity/buoyancy forces
          // MKC: This is SUCH a crappy design...
          if (m_material->buoyancy() > 0)
            f += m_buoyancyForce->apply(par, begin, end);
          else
            f += m_gravityForce->apply(par, begin, end);

        }
        //cout << "|normal|: " << mnorm << " " << norm/m_particles.size() << " " << Mnorm << endl;

        return true;
      }
#endif

      bool simulate()
      {
        if (!solve())
          return false;

        typename particle_container::iterator pbegin = m_particles.begin();
        typename particle_container::iterator pend = m_particles.end();

        m_integrator->integrate_particles(pbegin, pend);

#if defined(SPHSH)
        m_search.init_data(pbegin, pend);
#endif
        return true;
      }

    private:
      System& operator=(const System&){return *this;}

      void cleanUp()
      {
        delete m_density; m_density = NULL;
        delete m_pressure; m_pressure = NULL;
        delete m_normal; m_normal = NULL;
        delete m_gravityForce; m_gravityForce = NULL;
        delete m_buoyancyForce; m_buoyancyForce = NULL;
        delete m_pressureForce; m_pressureForce = NULL;
        delete m_viscosityForce; m_viscosityForce = NULL;
        delete m_surfaceForce; m_surfaceForce = NULL;
        delete m_integrator; m_integrator = NULL;
        delete m_color; m_color = NULL;
        m_particles.clear();
        m_cptr_particles.clear();
      }

      void incompressibility_relaxation(unsigned long iterations = 1)
      {
#if defined(SPHSH)
        particle_cptr_container particles;
#else
        particle_cptr_container &particles = m_cptr_particles;
#endif
        for (unsigned long n = 0; n < iterations; ++n)
          for (typename particle_container::iterator p = m_particles.begin(); p != m_particles.end(); ++p) {
            particle &par = *p;
#if defined(SPHSH)
            m_search(par, particles, typename fluid_point_query::all_tag() );
#endif
            // calc density
            par.density() = m_density->apply(par, particles);

            // calc pressure
            par.pressure() = m_pressure->apply(par, particles);

            vector dx = 0;
            if (par.density() > m_material->density())
              for (typename particle_cptr_container::const_iterator p_ = particles.begin(); p_ != particles.end(); ++p_) {
                const particle* p = *p_;
                vector d = p->position()-par.position();
                const real_type l = length(d);
                if (l <= 1./32768) continue;
                d *= 1./l;
                vector disp = 0.5*m_material->timestep()*m_material->timestep()*par.pressure()*(1-l/par.radius())*d;
                const_cast<particle*>(p)->position() += disp;
                dx -= disp;
              }
              par.position() += dx;
          }
      }

    protected:
      fluid_point_query  m_search;
      collision_detection  m_colisys;
      particle_container  m_particles;
      particle_cptr_container  m_cptr_particles;
      IntegratorPolicy*  m_integrator;
      const DensitySolver*  m_density;
      const PressureSolver*  m_pressure;
      const NormalSolver*  m_normal;
      const GravityForce*  m_gravityForce;
      const BuoyancyForce*  m_buoyancyForce;
      const PressureForce*  m_pressureForce;
      const ViscosityForce*  m_viscosityForce;
      const SurfaceForce*  m_surfaceForce;
      const ColorField*  m_color;
      const fluid_material*  m_material;

    }; // End class System

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_SYSTEM_H
#endif
