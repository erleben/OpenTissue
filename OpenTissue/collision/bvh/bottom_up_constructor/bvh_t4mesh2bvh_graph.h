#ifndef OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_T4MESH2BVH_GRAPH_H
#define OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_T4MESH2BVH_GRAPH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <iostream>
#include <vector>
#include <map>
#include <queue>

namespace OpenTissue
{

  namespace bvh
  {

    /**
    *  t4mesh to BVHGraph Convresion tool.
    *
    */
    template <typename mesh_type,typename graph_type>
    class T4Mesh2BVHGraph
    {
    public:

      //--- Convenience stuff
      typedef typename graph_type::node_ptr_type             node_ptr_type;
      typedef typename mesh_type::tetrahedron_type           tetrahedron_type;
      typedef typename mesh_type::tetrahedron_iterator       tetrahedron_iterator;

    protected:

      typedef typename std::queue<tetrahedron_type*>                 tetrahedron_ptr_queue;
      typedef typename std::map<tetrahedron_type *,node_ptr_type >    graph_node_map;

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
        for(tetrahedron_iterator tetrahedron=mesh.tetrahedron_begin();tetrahedron!=mesh.tetrahedron_end();++tetrahedron)
        {
          node_ptr_type node = graph.insert(&(*tetrahedron));
          m_lookup[&(*tetrahedron)] = node;
        }
        tetrahedron_ptr_queue Q;
        std::vector<bool> visited(mesh.size_tetrahedra());
        std::fill(visited.begin(),visited.end(),false);
        Q.push(&(*(mesh.tetrahedron(0))));
        while(!Q.empty())
        {
          tetrahedron_type * tetrahedron = Q.front();
          Q.pop();
          visited[tetrahedron->idx()] = true;

          tetrahedron_iterator i = tetrahedron->jkm();
          tetrahedron_iterator j = tetrahedron->kim();
          tetrahedron_iterator k = tetrahedron->ijm();
          tetrahedron_iterator m = tetrahedron->ikj();

          if(i!=mesh.tetrahedron_end() && !visited[i->idx()])
          {
            node_ptr_type A = m_lookup[tetrahedron];
            node_ptr_type B = m_lookup[&(*i)];
            graph.insert(A,B);
            Q.push(&(*i));
          }
          if(j!=mesh.tetrahedron_end() &&!visited[j->idx()])
          {
            node_ptr_type A = m_lookup[tetrahedron];
            node_ptr_type B = m_lookup[&(*j)];
            graph.insert(A,B);
            Q.push(&(*j));
          }
          if(k!=mesh.tetrahedron_end() && !visited[k->idx()])
          {
            node_ptr_type A = m_lookup[tetrahedron];
            node_ptr_type B = m_lookup[&(*k)];
            graph.insert(A,B);
            Q.push(&(*k));
          }
          if(m!=mesh.tetrahedron_end() && !visited[m->idx()])
          {
            node_ptr_type A = m_lookup[tetrahedron];
            node_ptr_type B = m_lookup[&(*m)];
            graph.insert(A,B);
            Q.push(&(*m));
          }
        }
        std::cout << "T4Mesh2BVHGraph::run() Graph with " << graph.size_nodes() << " nodes and  " << graph.size_edges() << " edges" << std::endl;
      }

    };

  } // namespace bvh

} // namespace OpenTissue

//OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_T4MESH2BVH_GRAPH_H
#endif
