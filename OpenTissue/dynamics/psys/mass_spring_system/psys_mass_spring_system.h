#ifndef OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_MASS_SPRING_SYSTEM_H
#define OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_MASS_SPRING_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/iterator/indirect_iterator.hpp>
#include <boost/bind.hpp>

#include <list>

namespace OpenTissue
{

  namespace psys
  {

    template<
      typename types
      , typename integrator_policy
    >
    class MassSpringSystem : public System<types>, public integrator_policy
    {
    public:

      typedef typename types::system_type                    system_type;
      typedef typename system_type::particle_iterator        particle_iterator;
      typedef typename system_type::const_particle_iterator  const_particle_iterator;

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;
      typedef typename types::constraint_type       constraint_type;
      typedef typename types::force_type            force_type;
      typedef typename types::geometry_holder_type  geometry_holder_type;
      typedef typename types::contact_point_type    contact_point_type;

    protected:

      unsigned int m_iterations;     ///< Maximum number of iterations of the constraint relaxation.
      bool         m_relaxation;     ///< Boolean flag indicating wheter contraint relaxation (shake and rattle) should be used.
      bool         m_projection;     ///< Boolean flag indicating wheter projection is used under relaxation to hande collisions with other objects.

    public:

      bool               & relaxation()       { return m_relaxation; }
      bool const         & relaxation() const { return m_relaxation; }
      bool               & projection()       { return m_projection; }
      bool const         & projection() const { return m_projection; }
      unsigned int       & iterations()       { return m_iterations; }
      unsigned int const & iterations() const { return m_iterations; }

    protected:

      typedef std::list<force_type *>         force_ptr_container;
      typedef std::list<geometry_holder_type> geometry_container;
      typedef std::list<constraint_type *>    constraint_ptr_container;

    public:

      typedef boost::indirect_iterator< typename force_ptr_container::iterator, force_type>           force_iterator;
      typedef boost::indirect_iterator< typename constraint_ptr_container::iterator, constraint_type> constraint_iterator;

      typedef typename geometry_container::iterator                                                   geometry_iterator;

    protected:

      force_ptr_container        m_forces;       ///< A list of forces.
      constraint_ptr_container   m_constraints;  ///< A list of constraints.
      geometry_container         m_geometries;   ///< A list of geometries that this particle cluster
                                                 ///< should perform collision detection against.
    public:

      force_iterator      force_begin()      { return force_iterator(m_forces.begin());           }
      force_iterator      force_end()        { return force_iterator(m_forces.end());             }
      constraint_iterator constraint_begin() { return constraint_iterator(m_constraints.begin());}
      constraint_iterator constraint_end()   { return constraint_iterator(m_constraints.end());  }
      geometry_iterator   geometry_begin()   { return geometry_iterator(m_geometries.begin());   }
      geometry_iterator   geometry_end()     { return geometry_iterator(m_geometries.end());     }

      void clear(void)
      {
        m_forces.clear();
        m_constraints.clear();
        m_geometries.clear();
        system_type::clear();
      };

      void add_force(force_type * F)              { F->connect(*this); m_forces.push_back(F);      }
      void remove_force(force_type * F)           { m_forces.remove(F); F->disconnect();           }

      void add_constraint(constraint_type * C)    { C->connect(*this); m_constraints.push_back(C); }
      void remove_constraint(constraint_type * C) { m_constraints.remove(C); C->disconnect();      }

      template<typename geometry_type>
      void add_geometry(geometry_type * G)
      {
        geometry_holder_type holder;
        holder.set(G);
        holder.connect(*this);
        m_geometries.push_back(holder);
      }

      template<typename geometry_type>
      void remove_geometry(geometry_type * G)
      {
        geometry_holder_type holder;
        holder.set(G);
        m_geometries.remove(holder);
      }

    public:

      MassSpringSystem(void)
        : m_iterations(10)
        , m_relaxation(true)
        , m_projection(true)
      {}

      ~MassSpringSystem(){  clear(); }

    public:

      void run(real_type timestep)
      {
        assert(timestep>0 || !"MassSpringSystem::run(): Non-positive time-step");
        integrator_policy::integrate ( *this, timestep ); //--- from integrator policy
        do_relaxation();
        this->time() += timestep;
      }

    protected:

      void do_relaxation()
      {
        if(!m_relaxation)
          return;

        for(unsigned int i=0;i<m_iterations;++i)
        {
          constraint_iterator c   = constraint_begin();
          constraint_iterator end = constraint_end();
          for(;c!=end;++c)
            c->satisfy();

          do_projection();
        }
      }

      void do_projection()
      {
        typedef std::list<contact_point_type> contact_point_container;

        if(!m_projection)
          return;

        contact_point_container contacts;


        {
          geometry_iterator g = geometry_begin();
          geometry_iterator end = geometry_end();
          for(;g!=end;++g)
            g->dispatch( *this, contacts );
          //        std::for_each(
          //            geometry_begin()
          //          , geometry_end()
          ////          ,  boost::bind( &geometry_type::dispatch , _1, *this , contacts )  // do not works?
          ////          ,  boost::bind( &geometry_type::test, _1, 2, 3)  // works
          ////          ,  boost::bind( &geometry_type::type, _1)        // works
          //          );
        }

        {
          typename std::list<contact_point_type>::iterator cp = contacts.begin();
          typename std::list<contact_point_type>::iterator end = contacts.end();
          for(;cp!=end;++cp)
          {
            if(cp->m_A0 && !cp->m_A1 && !cp->m_A2 && !cp->m_B0 && !cp->m_B1 && !cp->m_B2)
            {
              cp->m_A0->position() +=   cp->m_n*cp->m_distance;
            }


            if(cp->m_A0 && cp->m_A1 && !cp->m_A2 && cp->m_B0 && cp->m_B1 && cp->m_B2)
            {
              //--- Hack, I have not really thought about what to do?
              cp->m_A0->position() = cp->m_A0->old_position();
              cp->m_A1->position() = cp->m_A1->old_position();

              cp->m_A0->velocity().clear();
              cp->m_A1->velocity().clear();

              cp->m_B0->position() = cp->m_B0->old_position();
              cp->m_B1->position() = cp->m_B1->old_position();
              cp->m_B2->position() = cp->m_B2->old_position();

              cp->m_B0->velocity().clear();
              cp->m_B1->velocity().clear();
              cp->m_B2->velocity().clear();

            }

          }
        }
      }

    public:

      void compute_accelerations()
      {
        particle_iterator p   = this->particle_begin();
        particle_iterator end = this->particle_end();
        for(;p!=end;++p)
          p->acceleration() = p->force() * p->inv_mass();
      }

      void compute_forces()
      {
        particle_iterator p   = this->particle_begin();
        particle_iterator end = this->particle_end();
        for(;p!=end;++p)
          p->force().clear();

        std::for_each( force_begin(), force_end(), boost::bind( &force_type::apply, _1 ));
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_MASS_SPRING_SYSTEM_H
#endif
