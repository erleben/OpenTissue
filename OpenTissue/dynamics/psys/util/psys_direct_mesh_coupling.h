#ifndef OPENTISSUE_DYNAMICS_PSYS_UTIL_DIRECT_MESH_COUPLING_H
#define OPENTISSUE_DYNAMICS_PSYS_UTIL_DIRECT_MESH_COUPLING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <map>

namespace OpenTissue
{
  namespace psys
  {

    template< typename types >
    class DirectMeshCoupling
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;

    public:

      typedef typename types::mesh_type        mesh_type; 
      typedef typename mesh_type::vertex_type  vertex_type;

      typedef std::map<vertex_type*, particle_type*>         particle_lut_type;

    protected:

      particle_lut_type   m_particle_lut;         ///< Internal data structure used to find correspond particle of vertex.
      mesh_type         * m_mesh;                 ///< A pointer to the surface mesh.


    public:

      DirectMeshCoupling()
        : m_mesh(0)
      {}

    public:

      mesh_type       & mesh()       { return *m_mesh; }
      mesh_type const & mesh() const { return *m_mesh; }

      particle_type       & particle( vertex_type const & v )       {  return *m_particle_lut[ const_cast<vertex_type*>(&v) ]; }
      particle_type const & particle( vertex_type const & v ) const {  return *m_particle_lut[ const_cast<vertex_type*>(&v) ]; }


      /**
       *
       * Adds binder support for AABB Tree.
       *
       *
       * @param  v
       * @return 
       */
      particle_type       * operator()( vertex_type const * v )       {  return &(particle(*v)); }


    public:

      bool empty() const {  return !m_mesh; }

      void clear()
      {
        m_particle_lut.clear();
        m_mesh = 0;
      }

      template<typename particle_system_type>
        void init(particle_system_type & system, mesh_type & mesh)
      {      
        typedef typename particle_system_type::particle_iterator   particle_iterator;

        m_particle_lut.clear();
        system.clear();
        m_mesh = &mesh;

        typename mesh_type::vertex_iterator begin = mesh.vertex_begin();
        typename mesh_type::vertex_iterator end   = mesh.vertex_end();
        typename mesh_type::vertex_iterator v     = begin;

        for(;v!=end;++v)
          system.create_particle( particle_type() );

        particle_iterator p = system.particle_begin();
        for(v=begin;v!=end;++v,++p)
        {
          p->bind(v->m_coord);
          m_particle_lut[ &(*v) ] = &(*p);
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_UTIL_DIRECT_MESH_COUPLING_H
#endif
