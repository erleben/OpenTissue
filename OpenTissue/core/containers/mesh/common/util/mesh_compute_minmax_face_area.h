#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MINMAX_FACE_AREA_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MINMAX_FACE_AREA_H
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

    /**
    * Computes the maximum and minimum face area.
    *
    * @param mesh       The mesh that should be subdivided.
    * @param min_area   Upon return holds the maximum area.
    * @param max_area   Upon return holds the minimum area.
    */
    template<    typename mesh_type  >
    void compute_minmax_face_area( mesh_type const &  mesh, double & min_area, double & max_area  )
    {
      using std::max;
      using std::min;
      using std::fabs;

      typedef typename mesh_type::const_face_iterator            face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator   face_vertex_circulator;
      typedef typename mesh_type::math_types                     math_types;
      typedef typename math_types::value_traits                  value_traits;
      typedef typename math_types::vector3_type                  vector3_type;
      typedef typename math_types::real_type                     real_type;

      real_type max_area_sqr = 0;
      real_type min_area_sqr = math::detail::highest<real_type>();

      for(face_iterator f = mesh.face_begin();f!=mesh.face_end();++f)
      {
        face_vertex_circulator v0(*f);
        face_vertex_circulator v1(*f);++v1;
        face_vertex_circulator v2(*f);--v2;

        vector3_type const & p0 = v0->m_coord;
        vector3_type const & p1 = v1->m_coord;
        vector3_type const & p2 = v2->m_coord;
        vector3_type u1       = p2 - p1;
        vector3_type u2       = p1 - p0;
        vector3_type u1xu2    = u1 % u2;
        real_type area_sqr = u1xu2*u1xu2;
        max_area_sqr = max( max_area_sqr, area_sqr);
        min_area_sqr = min( min_area_sqr, area_sqr);
      }
      max_area = sqrt(max_area_sqr)*.5;
      min_area = sqrt(min_area_sqr)*.5;
      //std::cout << "mesh::compute_minmax_face_area(): min = " << min_area << " max = " << max_area << std::endl;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MINMAX_FACE_AREA_H
#endif
