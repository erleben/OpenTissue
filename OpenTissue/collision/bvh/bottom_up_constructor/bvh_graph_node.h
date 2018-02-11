#ifndef OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_NODE_H
#define OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_NODE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>


#include <cmath>       //Needed for fabs()
#include <list>        //Needed for graph data structure: edges, nodes and volumes
#include <iostream>    //Needed for debug output. NOTE: we should consider implementing a logging fascility!

namespace OpenTissue
{
  namespace bvh
  {

    template <typename bvh_type> class BVHGraph;///< Forward declaration.

    /**
    * A BVH Graph Node.
    */
    template <typename bvh_type>
    class BVHGraphNode
    {
    public:

      friend class BVHGraph<bvh_type>;

    public:

      typedef typename bvh_type::volume_type          volume_type;
      typedef typename bvh_type::bv_ptr          bv_ptr;
      typedef typename bvh_type::bv_ptr_container     bv_ptr_container;
      typedef typename bvh_type::geometry_container   geometry_container;
      typedef BVHGraph<bvh_type>                      graph_type;
      typedef typename graph_type::volume_container   volume_container;

    public:

      typedef typename graph_type::edge_ptr_container         edge_ptr_container;
      typedef typename graph_type::edge_ptr_iterator          edge_ptr_iterator;
      typedef typename graph_type::const_edge_ptr_iterator    const_edge_ptr_iterator;
      typedef typename graph_type::edge_iterator              edge_iterator;
      typedef typename graph_type::const_edge_iterator        const_edge_iterator;

      edge_iterator       edge_begin()       { return edge_iterator(m_edges.begin());       }
      edge_iterator       edge_end()         { return edge_iterator(m_edges.end());         }
      const_edge_iterator edge_begin() const { return const_edge_iterator(m_edges.begin()); }
      const_edge_iterator edge_end()   const { return const_edge_iterator(m_edges.end());   }

      edge_ptr_iterator       edge_ptr_begin()       { return m_edges.begin();       }
      edge_ptr_iterator       edge_ptr_end()         { return m_edges.end();         }
      const_edge_ptr_iterator edge_ptr_begin() const { return m_edges.begin();       }
      const_edge_ptr_iterator edge_ptr_end()   const { return m_edges.end();         }

      typedef typename graph_type::node_ptr_container            node_ptr_container;
      typedef typename graph_type::node_ptr_iterator             node_ptr_iterator;
      typedef typename graph_type::const_node_ptr_iterator       const_node_ptr_iterator;
      typedef typename graph_type::node_iterator                 node_iterator;
      typedef typename graph_type::const_node_iterator           const_node_iterator;

      //node_iterator       sub_node_begin()       { return node_iterator(m_sub_nodes.begin());       }
      //node_iterator       sub_node_end()         { return node_iterator(m_sub_nodes.end());         }
      //const_node_iterator sub_node_begin() const { return const_node_iterator(m_sub_nodes.begin()); }
      //const_node_iterator sub_node_end()   const { return const_node_iterator(m_sub_nodes.end());   }

      node_ptr_iterator       sub_node_ptr_begin()       { return m_sub_nodes.begin(); }
      node_ptr_iterator       sub_node_ptr_end()         { return m_sub_nodes.end();   }
      const_node_ptr_iterator sub_node_ptr_begin() const { return m_sub_nodes.begin(); }
      const_node_ptr_iterator sub_node_ptr_end()   const { return m_sub_nodes.end();   }

    protected:

      edge_ptr_container m_edges;           ///< Edges incidient to this node.
      node_ptr_container m_sub_nodes;       ///< Nodes in the region corresponding to this node (before a merge-operation)
      bv_ptr        m_bv;              ///< After a merge this member holds a pointer to the correspoding BV node.
      volume_type        m_volume;          ///< If the graph node has not yet been subject to a merge, this member contains a volume covering all the nodes contained by this node.
      unsigned int       m_height;          ///< Height of corresponding BV in the resulting BVH
      unsigned int       m_subtree_size;    ///< Number of BVs represented by this graph node.
      geometry_container m_coverage;        ///< Geometry covered by the BV graph node.

    public:

      unsigned int m_tag;                   ///< Tag, can be used for traversing graph structure.
      unsigned int m_tag2;                  ///< Tag, can be used for traversing graph structure.

    public:

      BVHGraphNode()
        : m_bv()
        , m_height(0)
        , m_subtree_size(0)
      {}

    public:

      /**
      * Create BV in BVH.
      *
      * @param bvh    A reference to the BVH where the BV should be created in.
      */
      void create_bv(bvh_type & bvh,bool const & annotated = false)
      {
        bv_ptr_container children;
        get_sub_node_BVs(children);
        m_bv = bvh.insert(children,annotated);
        ++m_subtree_size;
        m_bv->volume() = m_volume;
        if(size_sub_nodes())
          m_height = max_sub_node_height() + 1;
        else
          m_height = 0;
      }

      bv_ptr const & bv() const {return m_bv;}
      const unsigned int height()const{return m_height; }

      const unsigned int size_subtree()const{ return m_subtree_size; }
      const unsigned int size_edges()const{ return static_cast<unsigned int>(m_edges.size()); }
      const unsigned int size_sub_nodes()const{ return static_cast<unsigned int>(m_sub_nodes.size()); }

      geometry_container const & coverage() const { return m_coverage; }
      geometry_container & coverage(){ return m_coverage; }

      volume_type & volume()
      {
        if(m_bv)
          return m_bv->volume();
        return m_volume;
      }

      const unsigned int max_sub_node_height() const
      {
        using std::max;
        unsigned int max_height = 0;
        const_node_iterator node = m_sub_nodes.begin();
        const_node_iterator end  = m_sub_nodes.end();
        for(; node!=end; ++node)
          max_height = max(max_height, node->m_height);
        return max_height;
      }

      const unsigned int min_sub_node_height()const
      {
        using std::min;

        if(m_sub_nodes.empty())
          return 0;
        unsigned int min_height = math::detail::highest<unsigned int>();
        const_node_iterator node = m_sub_nodes.begin();
        const_node_iterator end  = m_sub_nodes.end();
        for(; node!=end; ++node)
          min_height = min(min_height, node->m_height);
        return min_height;
      }

      void get_sub_node_BVs(bv_ptr_container & bvs)
      {
        bvs.clear();
        const_node_iterator node( m_sub_nodes.begin() );
        const_node_iterator end( m_sub_nodes.end() );
        for(; node != end; ++node)
          bvs.push_back(node->m_bv);
      }

      void get_volumes(volume_container & volumes)
      {
        volumes.clear();
        const_node_iterator node( m_sub_nodes.begin() );
        const_node_iterator end( m_sub_nodes.end() );
        for(; node != end; ++node)
          if(node->m_bv)
            volumes.push_back((node->volume()));
      }

    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_NODE_H
#endif
