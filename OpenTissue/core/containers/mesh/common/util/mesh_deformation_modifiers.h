#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_DEFORMATION_MODIFIERS_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_DEFORMATION_MODIFIERS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/math/math_constants.h>

#include <cassert>
#include <cmath>

namespace OpenTissue
{
  namespace mesh
  {

    template<typename mesh_type,typename vector3_type>
    void translate(mesh_type & mesh,vector3_type const & translation)
    {
      typename mesh_type::vertex_iterator end = mesh.vertex_end();
      typename mesh_type::vertex_iterator   v = mesh.vertex_begin();
      for(;v!=end;++v)
        v->m_coord += translation;
    }

    template<typename mesh_type,typename matrix3x3_type>
    void rotate(mesh_type & mesh,matrix3x3_type const & R)
    {
      typename mesh_type::vertex_iterator end = mesh.vertex_end();
      typename mesh_type::vertex_iterator   v = mesh.vertex_begin();
      for(;v!=end;++v)
        v->m_coord = R*v->m_coord;
    }

    template<typename mesh_type,typename real_type>
    void uniform_scale(mesh_type & mesh,real_type const & s)
    {
      typename mesh_type::vertex_iterator end = mesh.vertex_end();
      typename mesh_type::vertex_iterator   v = mesh.vertex_begin();
      for(;v!=end;++v)
        v->m_coord = v->m_coord*s;
    }

    template<typename mesh_type,typename real_type>
    void scale(mesh_type & mesh,real_type const & sx,real_type const & sy,real_type const & sz)
    {
      typename mesh_type::vertex_iterator end = mesh.vertex_end();
      typename mesh_type::vertex_iterator   v = mesh.vertex_begin();
      for(;v!=end;++v)
      {
        v->m_coord(0) = v->m_coord(0)*sx;
        v->m_coord(1) = v->m_coord(1)*sy;
        v->m_coord(2) = v->m_coord(2)*sz;
      }
    }

    template<typename mesh_type,typename vector3_type>
    void scale(mesh_type & mesh,vector3_type const & s) { mesh::scale(mesh,s(0),s(1),s(2));  }

    template<typename mesh_type,typename vector3_type, typename real_type>
    void twist(mesh_type & mesh,vector3_type const & direction, real_type const & pitch)
    {
      OpenTissue::math::Matrix3x3<real_type> R;
      typename mesh_type::vertex_iterator end = mesh.vertex_end();
      typename mesh_type::vertex_iterator   v = mesh.vertex_begin();
      for(;v!=end;++v)
      {
        real_type projection = v->m_coord * direction;
        real_type radian = projection/pitch;
        R = Ru(radian,direction);
        v->m_coord = R * v->m_coord;
      }
    }

    template<typename mesh_type,typename vector3_type, typename real_type>
    void bend(mesh_type & mesh,vector3_type const & axis,vector3_type const & direction, real_type const & radius)
    {
      OpenTissue::math::Matrix3x3<real_type> R;
      real_type circum = 2*math::detail::pi<real_type>()*radius;
      typename mesh_type::vertex_iterator end = mesh.vertex_end();
      typename mesh_type::vertex_iterator   v = mesh.vertex_begin();
      for(;v!=end;++v)
      {
        real_type projection = v->m_coord * direction;
        real_type radian = projection/circum;
        R = Ru(radian,axis);
        v->m_coord = R * v->m_coord;
      }
    }

    template<typename mesh_type,typename vector3_type, typename real_type>
    void spherical_bend(mesh_type & mesh,vector3_type const & normal, real_type const & radius)
    {
      OpenTissue::math::Matrix3x3<real_type> R;
      real_type circum = 2*math::detail::pi<real_type>()*radius;
      vector3_type axis1,axis2;

      orthonormal_vectors(axis1,axis2,normal);

      typename mesh_type::vertex_iterator end = mesh.vertex_end();
      typename mesh_type::vertex_iterator   v = mesh.vertex_begin();
      for(;v!=end;++v)
      {
        real_type proj1 = v->m_coord*axis1;
        real_type proj2 = v->m_coord*axis2;
        real_type length = std::sqrt(proj1*proj1 + proj2*proj2);
        real_type radian = length/circum;
        vector3_type u = normalize(v->m_coord % normal);
        R = Ru(radian,u);
        v->m_coord = R * v->m_coord;
      }
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_DEFORMATION_MODIFIERS_H
#endif
