#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_FACE_AREA_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_FACE_AREA_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

#include <cmath>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Compute Face Area.
    *
    *
    * @param f       The face from which the area is computed.
    * @return        The face area.
    */
    template<typename mesh_type>
    typename mesh_type::math_types::real_type
      compute_face_area(PolyMeshFace<mesh_type> const & f)
    {
      using std::sqrt;

      typedef typename mesh_type::face_vertex_circulator   face_vertex_circulator;
      typedef typename mesh_type::vertex_type              vertex_type;

      typedef typename mesh_type::math_types               math_types;
      typedef typename math_types::vector3_type            vector3_type;
      typedef typename math_types::real_type               real_type;


      real_type area = real_type(); // by standard defualt consructed integral types are zero

      if(valency(f)<=2)
      {
        std::cout << "compute_face_area(): Face has less than three vertices!" << std::endl;
        return real_type();
      }

      face_vertex_circulator v(f),end;
      face_vertex_circulator vi(f);++vi;
      face_vertex_circulator vj(f);++vj;++vj;

      vector3_type A;
      A.clear();  // By OT convention, sets elements to zero!

      for(;vj!=end;++vi,++vj)
      {
        vector3_type ui       =  vi->m_coord - v->m_coord;
        vector3_type uj       =  vj->m_coord - v->m_coord;
        A    += (ui % uj);      
      }
      area = sqrt(A*A)*0.5;
      return area;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_FACE_AREA_H
#endif
