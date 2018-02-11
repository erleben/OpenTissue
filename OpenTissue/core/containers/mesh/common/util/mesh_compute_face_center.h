#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_CENTER_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_CENTER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mesh
  {
    template<typename face_type,typename vector3_type>
    void compute_face_center(face_type const & f, vector3_type & center)
    {
      typedef typename face_type::mesh_type                mesh_type;
      typedef typename mesh_type::face_vertex_circulator   face_vertex_circulator;
      
      typedef typename mesh_type::math_types               math_types;
      typedef typename math_types::real_type               real_type;

      center.clear();
      face_vertex_circulator v(f),end;
      for(;v!=end;++v)
        center += v->m_coord;
      center /= static_cast<real_type>(valency(f));
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_CENTER_H
#endif
