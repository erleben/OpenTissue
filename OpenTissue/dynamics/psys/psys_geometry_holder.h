#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_GEOMETRY_HOLDER_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_GEOMETRY_HOLDER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/collision/collision_psys_plane.h>
#include <OpenTissue/dynamics/psys/collision/collision_psys_sphere.h>
#include <OpenTissue/dynamics/psys/collision/collision_psys_sdf.h>


#include <OpenTissue/collision/collision_points_aabb_tree.h>
#include <OpenTissue/collision/collision_aabb_tree.h>


#include <cassert>

namespace OpenTissue
{
  namespace psys
  {


    /**
     *
     * Introducing the concept of a geometry holder seems to be a really good
     * idea. We no longer need to write ``geometry wrappers'' for each new type
     * of geometry we add to the particle system library. Instead we just need to
     * add a set-method to the geometry holder class and extend the dispatch method
     * with a corresponding entry.
     *
     * One more benefit is that we have completely removed the need for a particle
     * system to know anything about geometries...
     *
     * Observe further more that particle systems can not share instances of the geometry
     * holder, but they can easily share the actually geometry.
     *
     *
     * One major drawback remains, one have to edit existing code (the geometry holder)
     * when extending with new geometry types:-(
     *
     */
    template<typename types>
    class GeometryHolder
      : public types::connector_type
    {
    public:

      typedef enum {UNDEFINED, SPHERE, PLANE, SDF, AABB_TREE} type_index;

      typedef typename types::math_types             math_types;
      typedef typename math_types::real_type         real_type;
      typedef typename types::sphere_type            sphere_type;
      typedef typename types::plane_type             plane_type;
      typedef typename types::sdf_geometry_type      sdf_geometry_type;
      typedef typename types::aabb_tree_type         aabb_tree_type;

    protected:

      type_index m_type;
      void *     m_geometry;

    public:

      void* geometry()  { return m_geometry; }
      type_index const & type() { return m_type; }

    public:

      GeometryHolder()
        : m_type(UNDEFINED)
        , m_geometry(0)
      {}

    public:

      bool operator==(GeometryHolder const & geometry)
      {
        return (this->m_geometry == geometry.m_geometry);
      }

    public:

      void set(sphere_type * sphere)
      {
        m_type = SPHERE;
        m_geometry = static_cast<void*>(sphere);
      }

      void set(plane_type * plane)
      {
        m_type = PLANE;
        m_geometry = static_cast<void*>(plane);
      }

      void set(sdf_geometry_type * sdf)
      {
        m_type = SDF;
        m_geometry = static_cast<void*>(sdf);
      }


      void set(aabb_tree_type * aabb_tree)
      {
        m_type = AABB_TREE;
        m_geometry = static_cast<void*>(aabb_tree);
      }


      void clear()
      {
        m_type = UNDEFINED;
        m_geometry = 0;
      }

    public:

      template<typename particle_system, typename contact_point_container>
        void dispatch(particle_system & psys, contact_point_container & contacts)
      {
        if(m_type == SPHERE)
        {
          sphere_type * sphere = static_cast<sphere_type*>( m_geometry );
          collision_psys_sphere(psys,*sphere,contacts);
        }
        else if(m_type == PLANE)
        {
          plane_type * plane = static_cast<plane_type*>( m_geometry );
          collision_psys_plane(psys,*plane,contacts);
        }
        else if(m_type == SDF)
        {
          sdf_geometry_type * sdf = static_cast<sdf_geometry_type*>( m_geometry );
          collision_psys_sdf(psys,*sdf,contacts);
        }

        else if(m_type == AABB_TREE)
        {


          aabb_tree_type * aabb_tree = static_cast<aabb_tree_type*>( m_geometry );

          // TODO hmm, this may not be optimal? some kind of lazy refitting would be nice.
          OpenTissue::aabb_tree::refit(*aabb_tree);

          if( this->owner() == (&psys) )
            OpenTissue::collision::aabb_tree_against_itself( *aabb_tree, contacts );
          else
            OpenTissue::collision::points_aabb_tree( psys.particle_begin(), psys.particle_end(), *aabb_tree, contacts );
        }

        else
        {
          std::cerr << "GeometryHolder::dispatch(...): unrecognized geometry type?" << std::endl;
        }
      }

    };


  } // namespace psys

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PSYS_GEOMETRY_HOLDER_H
#endif
