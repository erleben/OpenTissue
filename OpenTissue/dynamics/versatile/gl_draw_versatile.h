#ifndef OPENTISSUE_DYNAMICS_VERSATILE_GL_DRAW_VERSATILE_H
#define OPENTISSUE_DYNAMICS_VERSATILE_GL_DRAW_VERSATILE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/gl/gl_util.h>

namespace OpenTissue
{
  namespace gl
  {

    template<typename versatile_mesh>
    inline void DrawVersatilePenaltyForces(versatile_mesh & mesh)
    {
      typename versatile_mesh::node_iterator begin = mesh.node_begin();
      typename versatile_mesh::node_iterator end   = mesh.node_end();
      typename versatile_mesh::node_iterator node;

      for(node=begin;node!=end;++node)
      {
        OpenTissue::gl::DrawVector(node->m_coord,node->m_f_pen);
      }
    }

    template<typename versatile_mesh>
    inline void DrawVersatileInternalForces(versatile_mesh & mesh)
    {
      typename versatile_mesh::node_iterator begin = mesh.node_begin();
      typename versatile_mesh::node_iterator end   = mesh.node_end();
      typename versatile_mesh::node_iterator node;

      for(node=begin;node!=end;++node)
      {
        OpenTissue::gl::DrawVector(node->m_coord,node->m_f_con);
      }
    }


    template<typename versatile_mesh>
    inline void DrawVersatileExternalForces(versatile_mesh & mesh)
    {
      typename versatile_mesh::node_iterator begin = mesh.node_begin();
      typename versatile_mesh::node_iterator end   = mesh.node_end();
      typename versatile_mesh::node_iterator node;

      for(node=begin;node!=end;++node)
      {
        OpenTissue::gl::DrawVector(node->m_coord,node->m_f_ext);
      }
    }

  } // namespace gl
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_GL_DRAW_VERSATILE_H
#endif
