#ifndef OPENTISSUE_COLLISION_VCLIP_VCLIP_MESH_H
#define OPENTISSUE_COLLISION_VCLIP_VCLIP_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>

#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_convex_hull.h>
#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_plane.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_vertex_edge_voronoi_plane.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_edge_face_voronoi_plane.h>


namespace OpenTissue
{
  namespace vclip
  {

    class Feature
    {
    public:

      typedef enum {UNDEFINED = -1, VERTEX = 0, EDGE = 1, FACE = 2} feature_type;

    public:

      feature_type  m_type;

    public:

      Feature(feature_type const & type) : m_type(type)
      {}

    };

    template<typename M>
    class VoronoiClipVertexTraits 
      : public mesh::DefaultVertexTraits<M>
      , public Feature
    {
    public:

      VoronoiClipVertexTraits() 
        : Feature(Feature::VERTEX)
      {}

    };

    template<typename M>
    class VoronoiClipHalfEdgeTraits 
      : public mesh::DefaultHalfEdgeTraits
      , public Feature
    {
    public:

      typedef          M                                       math_types;
      typedef typename math_types::real_type                   real_type;
      typedef typename math_types::vector3_type                vector3_type;
      typedef          geometry::Plane<math_types>             plane_type;

      plane_type m_voronoi_plane_VE;
      plane_type m_voronoi_plane_EF;

      vector3_type m_u;
      real_type    m_length;
      int          m_tag;

    public:

      VoronoiClipHalfEdgeTraits() 
        : Feature(Feature::EDGE)
      {}

    };

    class VoronoiClipEdgeTraits 
      : public mesh::DefaultEdgeTraits
      , public Feature
    {
    public:

      VoronoiClipEdgeTraits() 
        : Feature(Feature::UNDEFINED)
      {}

    };

    template<typename M>
    class VoronoiClipFaceTraits 
      : public mesh::DefaultFaceTraits
      , public Feature
    {
    public:

      typedef M                                       math_types;
      typedef geometry::Plane<math_types>             plane_type;

    public:

      plane_type m_plane;

    public:

      VoronoiClipFaceTraits() 
        : Feature(Feature::FACE)
      {}

    };

    template<typename M>
    class VClipMesh 
      : public polymesh::PolyMesh<
        M
      , VoronoiClipVertexTraits<M>
      , VoronoiClipHalfEdgeTraits<M>
      , VoronoiClipEdgeTraits
      , VoronoiClipFaceTraits<M>
      // , polymesh::PolyMeshListKernel
      >
    {};


    typedef VClipMesh< OpenTissue::math::BasicMathTypes<double,size_t> > vclip_mesh_type;

    /**
    * Update voronoi planes of all external voronoi regions of the given mesh.
    *
    * @param mesh
    */
    void update_voronoi_regions(vclip_mesh_type & mesh)
    {
      vclip_mesh_type::halfedge_iterator h    = mesh.halfedge_begin();
      vclip_mesh_type::halfedge_iterator hend = mesh.halfedge_end();
      for(;h!=hend;++h)
      {
        polymesh::compute_vertex_edge_voronoi_plane(  *(h->get_destination_iterator()), *h, h->m_voronoi_plane_VE );
        polymesh::compute_edge_face_voronoi_plane(  (*h), *(h->get_face_iterator()), h->m_voronoi_plane_EF );
        h->m_u = h->get_destination_iterator()->m_coord - h->get_origin_iterator()->m_coord;
        h->m_length = std::sqrt(h->m_u*h->m_u);
        h->m_u /= h->m_length;
        h->m_tag = -1;
      }
      vclip_mesh_type::face_iterator f = mesh.face_begin();
      vclip_mesh_type::face_iterator fend = mesh.face_end();
      for(;f!=fend;++f)
        mesh::compute_face_plane(*f,f->m_plane);
    }

    /**
    * Convert Utility.
    *
    * @param in
    * @param out
    *
    * @return
    */
    template< typename mesh_type >
    bool convert( mesh_type const & in, vclip_mesh_type & out)
    {
      typedef typename vclip_mesh_type::math_types     math_types;
      typedef typename math_types::vector3_type        vector3_type;

      unsigned int N = static_cast<unsigned int>(in.size_vertices());

      std::vector<vector3_type> points(N);

      typename mesh_type::const_vertex_iterator v = in.vertex_begin();

      for(unsigned int i=0;i<N;++i,++v)
        points[i] = v->m_coord;

      out.clear();
      mesh::convex_hull(points.begin(),points.end(),out);
      update_voronoi_regions(out);
      return true;
    }

  } // namespace vclip

} // namespace OpenTissue

//OPENTISSUE_COLLISION_VCLIP_VCLIP_MESH_H
#endif
