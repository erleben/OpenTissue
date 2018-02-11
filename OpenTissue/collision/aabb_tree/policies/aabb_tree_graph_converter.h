#ifndef OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_GRAPH_CONVERTER_H
#define OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_GRAPH_CONVERTER_H
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
  namespace aabb_tree
  {
    template <typename graph_type>
    class GraphConverter
    {
    public:

      typedef typename graph_type::geometry_type   geometry_type;
      typedef typename graph_type::volume_type     volume_type;
      typedef typename graph_type::node_ptr_type   node_ptr;

    public:

      /**
      * Run Mesh to Graph Conversion Method.
      *
      * @param mesh      The input mesh that should be converted to a graph data structure, that
      *                  can be used for bottom-up construction.
      * @param graph     Upon return this argument holds the generated graph.
      * @param binder    A user-specified functor that is used to look up the vertex_data that
      *                  should be bound to each triangle vertex. This could in fact be an
      *                  identity-mapping. However in some cases a mesh would be coupled to
      *                  for instance a particle system and it would make sense to bind the
      *                  particles as the vertices of the triangles.
      */
      template<
        typename mesh_type
        , typename vertex_data_binder
      >
      void run(mesh_type & mesh,  graph_type & graph, vertex_data_binder & binder)
      {
        typedef typename mesh_type::halfedge_type             halfedge_type;
        typedef typename mesh_type::face_type                 face_type;
        typedef typename mesh_type::vertex_type               vertex_type;
        typedef typename mesh_type::face_iterator             face_iterator;

        typedef typename std::queue<face_type*>               face_ptr_queue;
        typedef typename std::map<face_type *,node_ptr>       graph_node_map;

        graph_node_map lookup;   ///< Internally used data structure. Needed for looking up graph nodes corresponding to mesh faces.
        lookup.clear();

        face_iterator face = mesh.face_begin();
        face_iterator end  = mesh.face_end();
        for(;face!=end;++face)
        {
          geometry_type triangle;

          typename mesh_type::face_vertex_circulator v0(*face);
          typename mesh_type::face_vertex_circulator v1(*face);++v1;
          typename mesh_type::face_vertex_circulator v2(*face);++v2;++v2;

          //--- Ask user-specified binder what vertex data should be
          triangle.m_p0 = binder( &(*v0) );//  psys.m_particle_lut[ &(*v0) ];
          triangle.m_p1 = binder( &(*v1) );//  psys.m_particle_lut[ &(*v1) ];
          triangle.m_p2 = binder( &(*v2) );//  psys.m_particle_lut[ &(*v2) ];

          node_ptr node = graph.insert(  triangle  );
          lookup[ &(*face) ] = node;
        }

        face_ptr_queue Q;

        mesh::clear_face_tags(mesh);
        mesh::clear_halfedge_tags(mesh);

        Q.push( &(  *(mesh.face_begin())   )  );
        while(!Q.empty())
        {
          face_type * face = Q.front(); Q.pop();

          face->m_tag = 1;

          typename mesh_type::face_halfedge_circulator h(*face),hend;
          for(;h!=hend;++h)
            visist_connection( face, &(*h), Q, graph, lookup);

        }
        std::cout << "GraphConverter::run(): graph created with |N| = " 
          << graph.size_nodes() 
          << " and  |E| = " 
          << graph.size_edges() 
          << std::endl;
      }

    protected:

      /**
      * Auxiliary Method.
      * This method is used to visit neighboring faces of a given face.
      *
      * @param face      The current face
      * @param edge      An edge to the neighboring face we want to visit
      * @param Q         A queue holding all faces that we wish to visit.
      * @param graph     The graph data structure that is being constructed while
      *                  we visit edges to neighboring faces.
      * @param lookup    A graph node lookup table. This is used to find a graph
      *                  node that correspond to a given mesh face.
      */
      template< 
        typename face_type
        , typename halfedge_type
        , typename face_ptr_queue
        , typename lookup_map
      >
      void visist_connection(  face_type * face, halfedge_type * edge,  face_ptr_queue & Q, graph_type & graph, lookup_map & lookup  )
      {
        //--- See if we have a connection we have not yet visited
        if(edge->get_twin_handle().is_null())
          return;

        if(edge->m_tag == 1)
          return;

        halfedge_type * twin_edge = &(*edge->get_twin_iterator());

        if(twin_edge->get_face_handle().is_null())
          return;

        face_type * twin_face = &(*(twin_edge->get_face_iterator()));

        node_ptr A = lookup[face];
        node_ptr B = lookup[twin_face];

        assert(A!=B);

        edge->m_tag = 1;
        twin_edge->m_tag = 1;

        graph.insert(A,B);

        if(twin_face->m_tag == 0)
          Q.push(twin_face);
      }

    };

  } // namespace aabb_tree
} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_GRAPH_CONVERTER_H
#endif
