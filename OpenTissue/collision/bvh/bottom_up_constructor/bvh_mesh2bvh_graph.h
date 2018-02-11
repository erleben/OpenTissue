#ifndef OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_MESH2BVH_GRAPH_H
#define OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_MESH2BVH_GRAPH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <iostream>
#include <queue>
#include <map>
#include <OpenTissue/core/containers/mesh/mesh.h>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    *  Mesh to BVHGraph Convresion tool.
    *
    */
    template <typename graph_type>
    class Mesh2BVHGraph
    {
    public:

      //--- Convenience stuff
      typedef typename graph_type::volume_type                 volume_type;
      typedef typename graph_type::node_ptr_type               node_ptr_type;
      typedef typename OpenTissue::polymesh::PolyMesh<>        mesh_type;
      typedef typename mesh_type::halfedge_type                halfedge_type;
      typedef typename mesh_type::face_type                    face_type;
      typedef typename mesh_type::face_iterator                face_iterator;

    protected:

      typedef typename std::queue<face_type*>                 face_ptr_queue;
      typedef typename std::map<face_type *,node_ptr_type>    graph_node_map;

    protected:

      graph_node_map m_lookup;   ///< Internally used data structure. Needed for looking up graph nodes corresponding to mesh faces.

    public:

      /**
      * Run algorithm.
      *
      * @param mesh
      * @param graph
      */
      void run(mesh_type & mesh,graph_type & graph)
      {
        m_lookup.clear();

        for(face_iterator face=mesh.face_begin();face!=mesh.face_end();++face)
        {
          node_ptr_type node = graph.insert(&(*face));
          m_lookup[&(*face)] = node;
        }
        face_ptr_queue Q;

        mesh::clear_face_tags(mesh);
        mesh::clear_halfedge_tags(mesh);

        Q.push( &(*mesh.face_begin()) );
        while(!Q.empty())
        {
          face_type * face = Q.front();
          Q.pop();

          face->m_tag = 1;

          mesh_type::face_halfedge_circulator h(*face),hend;
          for(;h!=hend;++h)
            visistConnection(face, &(*h), Q, graph);
        }
        std::cout << "Mesh2BVHGraph::run(): Graph with " << graph.size_nodes() << " nodes and  " << graph.size_edges() << " edges" << std::endl;
      }

    protected:

      void visistConnection(  face_type * face, halfedge_type * edge,  face_ptr_queue & Q, graph_type & graph  )
      {
        //--- See if we have a connection we have not yet visited
        if(edge->get_twin_handle().is_null())
          return;

        if(edge->m_tag==1)
          return;

        halfedge_type * twin_edge = &(*edge->get_twin_iterator());

        if(twin_edge->get_face_handle().is_null())
          return;

        face_type * twin_face = &(*twin_edge->get_face_iterator());

        node_ptr_type A = m_lookup[face];
        node_ptr_type B = m_lookup[twin_face];

        assert(A!=B);

        edge->m_tag = 1;
        twin_edge->m_tag = 1;

        graph.insert(A,B);

        if(twin_face->m_tag==0)
          Q.push(twin_face);
      }

    };

  } // namespace bvh

} // namespace OpenTissue

//OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_MESH2BVH_GRAPH_H
#endif
