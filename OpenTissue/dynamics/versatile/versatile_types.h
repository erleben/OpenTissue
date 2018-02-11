#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_TYPES_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_TYPES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/versatile/versatile_mesh.h>
#include <OpenTissue/dynamics/versatile/versatile_simulator.h>

namespace OpenTissue
{
  namespace versatile
  {

    template<typename math_types>
    class Types 
      : public math_types
    {
    public:

      typedef Types<math_types>                                 versatile_types;

      typedef OpenTissue::versatile::Mesh<versatile_types>      mesh_type;
      typedef typename mesh_type::node_type                     node_type;
      typedef typename mesh_type::tetrahedron_type              tetrahedron_type;
      typedef OpenTissue::versatile::Simulator<versatile_types> simulator_type;
    };


  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_TYPES_H
#endif
