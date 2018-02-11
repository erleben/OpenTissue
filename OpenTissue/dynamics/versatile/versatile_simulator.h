#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_SIMULATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/versatile/versatile_mesh.h>
#include <OpenTissue/dynamics/versatile/versatile_collision_policy.h>
#include <OpenTissue/collision/spatial_hashing/spatial_hashing.h>
#include <OpenTissue/core/geometry/geometry_plane.h>

#include <boost/iterator/indirect_iterator.hpp>

#include <list>

namespace OpenTissue
{
  namespace versatile
  {

    /**
    * Versatile Method.
    *
    * Some observations:
    *
    *   Good stability  <- small time steps, small geometries (re-scale world to unity), keep k_D, k_A, k_V below 100 or so.
    *
    */
    template< typename versatile_types >
    class Simulator
    {
    public:

      typedef typename versatile_types::value_traits                 value_traits;
      typedef typename versatile_types::real_type                    real_type;
      typedef typename versatile_types::vector3_type                 vector3_type;
      typedef typename versatile_types::matrix3x3_type               matrix3x3_type;

      typedef OpenTissue::versatile::Mesh<versatile_types>           mesh_type;
      typedef typename std::list<mesh_type*>                         mesh_ptr_container;
      typedef typename mesh_ptr_container::iterator                  mesh_ptr_iterator;
      typedef boost::indirect_iterator<mesh_ptr_iterator,mesh_type>  mesh_iterator;

      typedef OpenTissue::versatile::detail::collision_policy<versatile_types> policy;

      typedef OpenTissue::spatial_hashing::PointDataQuery<typename policy::hash_grid,policy>  point_query_type;

      typedef typename policy::result_container                      contact_container;
      typedef typename contact_container::iterator                   contact_iterator;
      typedef OpenTissue::geometry::Plane<versatile_types>           plane_type;


    protected:

      mesh_ptr_container     m_meshes;       ///< All objects that should be simulated.
      contact_container      m_contacts;     ///< Container holding collision detection results.
      point_query_type       m_point_query;  ///< The collision detection.

    public:

      void add(mesh_type & mesh)
      {
        m_point_query.auto_init_settings(mesh.tetrahedron_begin(),mesh.tetrahedron_end());//--- KE 02-07-2005: Hmmm, may not the best place to do this?
        m_meshes.push_back(&mesh);
      }

      void clear() { m_meshes.clear(); }

      /**
      * Run Simulation.
      * Remeber to compute external forces prior to invoking this method.
      *
      * @param dT           The frame time.
      * @param fraction     Error reduction parameter, indicates the fraction that
      *                     penetrations should be reduced too within time dT. Don't be
      *                     too overeager, value of 0.05 (5%) seems to do a good job.
      */
      void run(real_type const & dT,real_type const & fraction)
      {
        using std::min;
        using std::ceil;
        using std::log;

        assert(dT>value_traits::zero()                                            || !"run(): Frame time-step must be positive");
        assert( (fraction>value_traits::zero() && fraction<=value_traits::one() ) || !"run(): penetration reduction parameter must be wihtin (0..1]");

        //--- Estimate time step size which will keep the method stable.
        //---
        //--- Currently I only consider the penalty forces from
        //--- contacts, I should also consider the distance, area
        //--- and volume constraints.
        //---
        //--- However, it appears that penalty forces are more stiff than
        //--- the preservation constraints?
        //---
        //---

        real_type const min_dt = boost::numeric_cast<real_type>( 0.001 );

        real_type n    =  ceil( - log( fraction ));             //--- number of steps to take so procenwise error drops below fraction.
        real_type tau  = dT/n;                                  //--- characteristic time step size.
        real_type mu   = value_traits::one();                   //--- maximum mass in configuration (I just hardwired this for now).
        real_type b = (value_traits::two()*mu)/tau;             //--- maximum possible damping coefficient of penalty force.
        real_type dt = min(min_dt,value_traits::four()*mu/b);   //--- Step size of Verlet Integrator...
        int N = boost::numeric_cast<int>( ceil(dT/dt) );        //--- Number of Verlet steps to take...

        collision_detection();

        for(int i=0;i<N;++i)
          run(dT,fraction,dt);
      }

      real_type compute_internal_energy()
      {
        real_type energy = value_traits::zero();
        mesh_iterator begin = m_meshes.begin();
        mesh_iterator end   = m_meshes.end();
        mesh_iterator mesh;
        for(mesh=begin;mesh!=end;++mesh)
          energy += mesh->compute_internal_energy();
        return energy;
      }


    protected:

      /**
      * Time-Step method.
      *
      * @param dT        Frame time-step.
      * @param fraction  Penetration reduction parameter.
      * @param dt        The time-step time. Observe that an explicit integration
      *                  method (Verlet) is being used. Your time-step should be
      *                  order of ms (milliseconds) otherwise the simulation
      *                  becomes unstable.
      */
      void run(real_type const & dT,real_type const & fraction,real_type const & dt)
      {
        assert(dT>value_traits::zero()                                             || !"run(): Frame time-step must be positive");
        assert( ( fraction>value_traits::zero() && fraction<=value_traits::one() ) || !"run(): penetration reduction parameter must be wihtin (0..1]");
        assert(dt>value_traits::zero()                                             || !"run(): simulation time-step must be positive");
        assert(dt<dT                                                               || !"run(): simulation time-step must be smaller than frame time-step");

        mesh_iterator begin = m_meshes.begin();
        mesh_iterator end   = m_meshes.end();
        mesh_iterator mesh;

        for(mesh=begin;mesh!=end;++mesh)
          mesh->clear_constraint_forces();

        collision_resolving(dT,fraction);

        for(mesh=begin;mesh!=end;++mesh)
          mesh->apply_constraint_forces();

        for(mesh=begin;mesh!=end;++mesh)
          mesh->integrate(dt);
      }

      void collision_resolving(real_type const & dT,real_type const & fraction)
      {
        using std::ceil;
        using std::log;

        plane_type plane[4];
        real_type  d[4];

        assert(dT>value_traits::zero()                                             || !"collision_resolving(): Frame time-step must be positive");
        assert( ( fraction>value_traits::zero() && fraction<=value_traits::one() ) || !"collision_resolving(): penetration reduction parameter must be wihtin (0..1]");

        real_type n    =  ceil( - log( fraction ));
        real_type tau  = dT/n;
        {
          mesh_iterator begin = m_meshes.begin();
          mesh_iterator end   = m_meshes.end();
          mesh_iterator mesh;
          for(mesh=begin;mesh!=end;++mesh)
            mesh->clear_penalty_forces();
        }
        contact_iterator begin = m_contacts.begin();
        contact_iterator end   = m_contacts.end();
        contact_iterator contact;
        for(contact=begin;contact!=end;++contact)
        {
          vector3_type & ni = contact->m_query->i()->m_coord;
          vector3_type & nj = contact->m_query->j()->m_coord;
          vector3_type & nk = contact->m_query->k()->m_coord;
          vector3_type & nm = contact->m_query->m()->m_coord;
          vector3_type & p  = contact->m_data->m_coord;
          vector3_type & v  = contact->m_data->m_v;
          plane[0].set(nk,nj,ni);
          plane[1].set(ni,nj,nm);
          plane[2].set(nj,nk,nm);
          plane[3].set(nk,ni,nm);
          for(unsigned int i=0;i<4;++i)
            d[i] = plane[i].signed_distance(p);

          //--- This way of finding contact normal and penetration depth really sucks.
          //--- Sometimes the wrong plane is picked and an object is sucked into another
          //--- object:-(((
          //---
          //--- However, it is a really good idea to try to recompute contact normals
          //--- and penetration depths, because the collision detection engine is run
          //--- less frequently than the collision resolving.

          real_type max_d = d[0];
          int max_i = 0;
          for(int i=1;i<4;++i)
            if (d[i]>max_d)
            {
              max_d = d[i];
              max_i = i;
            }

            if( max_d>value_traits::zero() )
              continue;

            real_type mu   = contact->m_data->m_mass;
            real_type b = (value_traits::two()*mu)/tau;
            real_type k = mu/(tau*tau);

            vector3_type & n  = plane[max_i].n();
            vector3_type & f  = contact->m_data->m_f_pen;

            f += k*n*max_d - b*(v*n)*n;
        }
      }
      void collision_detection()
      {
        mesh_iterator begin = m_meshes.begin();
        mesh_iterator end   = m_meshes.end();
        mesh_iterator mesh;
        m_contacts.clear();
        for(mesh=begin;mesh!=end;++mesh)
          m_point_query.init_data(mesh->node_begin(),mesh->node_end());
        for(mesh=begin;mesh!=end;++mesh)
          m_point_query(mesh->tetrahedron_begin(),mesh->tetrahedron_end(),m_contacts, typename point_query_type::all_tag());
        std::cout << "|C| = " << m_contacts.size() << std::endl;
      }

    };

  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_SIMULATOR_H
#endif
