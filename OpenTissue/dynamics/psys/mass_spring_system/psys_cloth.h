#ifndef OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_CLOTH_H
#define OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_CLOTH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/mass_spring_system/psys_surface_mesh.h>

#include <boost/multi_array.hpp>

namespace OpenTissue
{
  namespace psys
  {

    template<
      typename types
      , typename integrator_policy
    >
    class Cloth : public SurfaceMesh<types,integrator_policy>
    {
    public:

      typedef SurfaceMesh<types,integrator_policy> base_class;
      typedef typename types::coupling_type                     coupling_type;
      typedef typename types::mesh_type                         mesh_type;
      typedef typename mesh_type::vertex_handle                 vertex_handle;
      typedef typename types::math_types                        math_types;
      typedef typename math_types::real_type                    real_type;
      typedef typename math_types::vector3_type                 vector3_type;

    protected:

      mesh_type m_cloth;   ///< The Cloth Mesh.

    public:

      Cloth() {}

      virtual ~Cloth() {}

    public:

      // Overwrites init in SurfaceMesh
      void init(mesh_type /*const */ & mesh, bool create_sticks, bool create_springs)
      {
        base_class::init(mesh,create_sticks,create_springs);
      }

      void init(
          real_type const & width
        , real_type const & height
        , unsigned int subdivisions_width
        , unsigned int subdivisions_height
        , bool create_sticks
        , bool create_springs
        )
      {
        assert( subdivisions_width > 1   || !"Cloth::init(): Subdivision width must be larger than 1");
        assert( subdivisions_height > 1  || !"Cloth::init(): Subdivision height must be larger than 1");
        assert( width>0                  || !"Cloth::init(): Width must be positive");
        assert( height>0                 || !"Cloth::init(): Height must be positive");

        this->rigidty() = 2;

        real_type dx = width /(subdivisions_width-1);
        real_type dy = height /(subdivisions_height-1);
        mesh::make_plane(dx,dy,subdivisions_width,subdivisions_height,this->m_cloth);

        base_class::init(m_cloth,create_sticks,create_springs);
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_CLOTH_H
#endif
