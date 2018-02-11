#ifndef OPENTISSUE_COLLISION_SDF_SDF_COMPUTE_POINT_SAMPLING_H
#define OPENTISSUE_COLLISION_SDF_SDF_COMPUTE_POINT_SAMPLING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/mesh.h>

#include <boost/cast.hpp> //--- Needed for boost::numerical_cast

#include <list>

namespace OpenTissue
{
  namespace sdf
  {

    /**
    * Compute Point Sampling.
    * This function tries to resample a mesh geometry to better fit the
    * resolution of the corresponding signed distance map.
    *
    * @param mesh               The surface mesh from which a point sampling is computed.
    * @param phi                The signed distance field corresponding to the specified mesh.
    *
    * @param edge_resolution    Threshold value, indicating the sampling
    *                           resolution along edges. If zero it will be
    *                           computed on the fly, to match the resolution
    *                           of the signed distance map.
    *
    * @param face_sampling      Boolean flag indicating wheter face sampling is on or off.
    *
    * @param points             Upon return this argument holds the computed point sampling.
    */
    template<typename mesh_type,typename grid_type, typename point_container>
    void compute_point_sampling(
      mesh_type /*const*/ & mesh
      , grid_type const & phi
      , double edge_resolution
      , bool face_sampling
      , point_container & points
      )
    {
      using std::min;
      using std::max;
      using std::sqrt;

      typedef typename mesh_type::vertex_iterator             vertex_iterator;
      typedef typename mesh_type::halfedge_iterator           halfedge_iterator;
      typedef typename mesh_type::face_iterator               face_iterator;
      typedef typename mesh_type::face_type                   face_type;
      typedef typename mesh_type::halfedge_type               halfedge_type;
      typedef typename mesh_type::face_halfedge_circulator    face_halfedge_circulator;
      typedef typename std::list<face_type*>                  face_queue;

      typedef typename mesh_type::math_types                  math_types;
      typedef typename math_types::vector3_type               vector3_type;
      typedef typename math_types::real_type                  real_type;

      assert(edge_resolution>=0 || !"compute_point_sampling(): edge resolution was negative");

      mesh::clear_vertex_tags( mesh);
      mesh::clear_halfedge_tags( mesh);
      mesh::clear_face_tags( mesh);

      points.clear();

      //--- Ignore vertices in flat regions
      for(vertex_iterator v = mesh.vertex_begin();v!=mesh.vertex_end();++v)
      {
        v->m_tag = 1;
        if(!is_convex(  *v  ) )
          continue;
        points.push_back( v->m_coord );
      }
      //--- long flat edges are linearly sub-samplet, to help catch edge-face intersections.


      real_type tmp = boost::numeric_cast<real_type>( edge_resolution );
      real_type threshold = max(tmp, sqrt( phi.dx()*phi.dx() + phi.dy()*phi.dy() + phi.dz()*phi.dz() ));

      for(halfedge_iterator h = mesh.halfedge_begin();h!=mesh.halfedge_end();++h)
      {
        if(h->m_tag)
          continue;
        h->m_tag = 1;
        h->get_twin_iterator()->m_tag = 1;
        if(!is_convex( *h ) )
          continue;
        vector3_type u = h->get_destination_iterator()->m_coord - h->get_origin_iterator()->m_coord;
        real_type lgth = sqrt(u*u);
        if(lgth>threshold)
        {
          u /= lgth;
          vector3_type p = h->get_origin_iterator()->m_coord;
          real_type t = threshold;
          while(t<lgth)
          {
            p += u*threshold;
            t += threshold;
            points.push_back( p );
          }
        }
      }

      //--- Objects perfectly aligned along flat faces, may penetrate, to
      //--- avoid this, centroid point of flat regions are added to sample points.
      if(face_sampling)
      {
        vector3_type Ai,ai,ei;
        real_type area_test = max(  phi.dx()*phi.dy(), max(phi.dx()*phi.dz(),phi.dy()*phi.dz()));

        for(face_iterator face = mesh.face_begin();face!=mesh.face_end();++face)
        {
          if(face->m_tag)
            continue;
          real_type area = 0;
          vector3_type centroid = vector3_type(0,0,0);
          unsigned int size = 0;
          face_queue Q;
          Q.push_back( &(*face) );
          face->m_tag = 1;
          while(!Q.empty())
          {
            face_type * cur = Q.front();Q.pop_front();
            face_halfedge_circulator h(*cur),hend;
            for(;h!=hend;++h)
            {
              ai = h->get_origin_iterator()->m_coord;
              ei = h->get_destination_iterator()->m_coord - ai;
              Ai = ai % ei;
              area += 0.5*sqrt(Ai*Ai);
              ++size;
              centroid += h->get_origin_iterator()->m_coord;

              if(h->get_twin_iterator()->get_face_handle().is_null())
                continue;

              face_type * neighbor = &(*h->get_twin_iterator()->get_face_iterator());
              bool unseen = !neighbor->m_tag;
              // TODO 2007-02-08: polymesh specific, bad idea
              bool coplanar = is_planar(*h);  
              if(unseen && coplanar)
              {
                neighbor->m_tag = 1;
                Q.push_back(neighbor);
              }
            }
          }
          if(size && area > area_test)
          {
            centroid /= size;
            points.push_back( centroid );
          }
        }
      }
    }

  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_COMPUTE_POINT_SAMPLING_H
#endif
