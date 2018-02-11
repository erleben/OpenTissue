#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_PLANE_CLIPPER_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_PLANE_CLIPPER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/intersect/intersect_line_plane.h>

#include <utility>  //--- needed for std::pair
#include <list>
#include <map>

namespace OpenTissue
{
  namespace mesh
  {
    /**
    * Mesh Plane Clipper Function.
    *
    * Only works on meshes with convex faces!!!
    *
    * @param mesh          A reference to the mesh that should be cut by a plane.
    * @param plane         A reference to the cutting plane.
    * @param above         A reference to a mesh, which upon return contains the half
    *                      of the mesh that was lying above the cutting plane.
    * @param below         A reference to a mesh, which upon return contains the half
    *                      of the mesh that was lying below the cutting plane.
    * @param tolerance     A threshold value used for to handle problems with numerical precision (nonnegative value).
    */
    template<
      typename mesh_type
      , typename plane_type
    >
    void plane_clipper( mesh_type const & mesh, plane_type plane, mesh_type & above, mesh_type & below, double const & tolerance = 10e-6  )
    {
      typedef typename mesh_type::index_type                    index_type;
      typedef typename mesh_type::vertex_handle                 vertex_handle;
      typedef typename mesh_type::face_handle                   face_handle;
      typedef typename mesh_type::const_vertex_iterator         const_vertex_iterator;
      typedef typename mesh_type::const_face_iterator           const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator  const_face_vertex_circulator;

      typedef typename mesh_type::math_types                    math_types;
      typedef typename math_types::value_traits                 value_traits;
      typedef typename math_types::vector3_type                 vector3_type;
      typedef typename math_types::real_type                    real_type;

      assert(tolerance>=0 || !"tolerance was negative!!!");

      above.clear();
      below.clear();

      //--- First compute signed distances of all vertices wrt. the cut plane
      std::vector<real_type> distances(mesh.size_vertices(),static_cast<real_type>(0.0));   //--- holds signed distances from all the vertices in mesh to the cutting plane

      const_vertex_iterator vend = mesh.vertex_end();
      const_vertex_iterator v    = mesh.vertex_begin();
      for(;v!=vend;++v)
      {
        index_type idx = v->get_handle().get_idx();
        distances[idx] = plane.signed_distance(v->m_coord);
        if( std::fabs(distances[idx]) <= tolerance)
          distances[idx] = 0;
      }

      std::vector<vertex_handle>    v2a_lut(mesh.size_vertices());
      std::vector<vertex_handle>    v2b_lut(mesh.size_vertices());

      typedef std::pair<vertex_handle,vertex_handle> edge_key_type;
      std::map<edge_key_type,vertex_handle>    e2a_lut;
      std::map<edge_key_type,vertex_handle>    e2b_lut;

      const static unsigned int is_on    = 0;
      const static unsigned int is_above = 1;
      const static unsigned int is_below = 2;

      const_face_iterator fend = mesh.face_end();
      const_face_iterator f    = mesh.face_begin();

      for(;f!=fend;++f)
      {
        unsigned int state = is_on;

        const_face_vertex_circulator cur(*f);
        for(unsigned int i=0;i<valency(*f);++i,++cur)
        {
          if(distances[cur->get_handle().get_idx()]>0)
          {
            state = is_above;
            break;
          }
          if(distances[cur->get_handle().get_idx()]<0)
          {
            state = is_below;
            break;
          }
        }
        if(state==is_on)
          continue;

        real_type zero = static_cast<real_type>(0.0);
        std::list<vertex_handle> handles_above;
        std::list<vertex_handle> handles_below;

        const_face_vertex_circulator next(cur);++next;
        for(unsigned int i=0;i<valency(*f);++i,++cur,++next)
        {
          index_type cur_idx = cur->get_handle().get_idx();
          index_type next_idx = next->get_handle().get_idx();

          edge_key_type e_key = std::make_pair(std::min(cur_idx,next_idx),std::max(cur_idx,next_idx) );

          real_type O = distances[cur_idx];
          real_type D = distances[next_idx];

          bool cur_is_above = (O>zero);
          bool cur_is_below = (O<zero);
          bool cur_is_on    = !(cur_is_above || cur_is_below);
          bool next_is_above = (D>zero);
          bool next_is_below = (D<zero);
          bool next_is_on    = !(next_is_above || next_is_below);

#define add_cur_to_above() \
  if(v2a_lut[cur_idx].is_null()) \
  v2a_lut[cur_idx] = above.add_vertex(cur->m_coord);  \
  handles_above.push_back( v2a_lut[cur_idx]  );

#define add_cur_to_below() \
  if(v2b_lut[cur_idx].is_null()) \
  v2b_lut[cur_idx] = below.add_vertex(cur->m_coord);  \
  handles_below.push_back( v2b_lut[cur_idx]  );

#define add_crossing_to_above(p) \
  if(e2a_lut[e_key].is_null()) \
  e2a_lut[e_key] = above.add_vertex(p); \
  handles_above.push_back( e2a_lut[e_key]  );

#define add_crossing_to_below(p) \
  if(e2b_lut[e_key].is_null()) \
  e2b_lut[e_key] = below.add_vertex(p); \
  handles_below.push_back( e2b_lut[e_key]  );

          if (cur_is_on &&  next_is_below &&  state == is_above)
          {
            add_cur_to_above();
            add_cur_to_below();
            state = is_below;
          }
          else if (cur_is_on   && next_is_below   && state == is_below)
          {
            add_cur_to_below();
            state = is_below;
          }
          else if (cur_is_on  &&  next_is_above &&  state == is_above)
          {
            add_cur_to_above();
            state = is_above;
          }
          else if (cur_is_on && next_is_above &&  state == is_below)
          {
            add_cur_to_above();
            add_cur_to_below();
            state = is_above;
          }
          else if (cur_is_above && (next_is_above || next_is_on))
          {
            add_cur_to_above();
            state = is_above;
          }
          else if (cur_is_below && (next_is_below|| next_is_on) )
          {
            add_cur_to_below();
            state = is_below;
          }
          else if (cur_is_above && next_is_below)
          {
            add_cur_to_above();
            vector3_type p;
            OpenTissue::intersect::line_plane(cur->m_coord,next->m_coord,plane,p);

            add_crossing_to_above(p);
            add_crossing_to_below(p);
            state = is_below;
          }
          else if (cur_is_below &&  next_is_above)
          {
            add_cur_to_below();
            vector3_type p;
            OpenTissue::intersect::line_plane(cur->m_coord,next->m_coord,plane,p);
            add_crossing_to_below(p);
            add_crossing_to_above(p);
            state = is_above;
          }
          else if (cur_is_on && next_is_on && state == is_above)
          {
            add_cur_to_above();
          }
          else if (cur_is_on && next_is_on && state == is_below)
          {
            add_cur_to_below();
          }

#undef add_cur_to_above
#undef add_cur_to_below
#undef add_crossing_to_above
#undef add_crossing_to_below

        }
        assert( !(handles_above.empty() && handles_below.empty()) || !"Oups no cut faces?");
        //--- In case we didn't care about keeping faces triangular, simply just do this:
        if(!handles_above.empty())
          above.add_face(handles_above.begin(),handles_above.end());
        if(!handles_below.empty())
          below.add_face(handles_below.begin(),handles_below.end());
        //--- make sure we have cleaned up after us.
        handles_above.clear();
        handles_below.clear();
      }
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_PLANE_CLIPPER_H
#endif
