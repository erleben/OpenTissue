#ifndef OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_BINARY_MATCHING_BOTTOM_UP_POLICY_H
#define OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_BINARY_MATCHING_BOTTOM_UP_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_graph.h>
#include <list>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * Trying to match nodes into pairs, which must all have been
    * collapsed before a new matching can be done.
    */
    template<typename bvh_type>
    class BinaryMatchBottomUpPolicy
    {
    public:


      typedef BVHGraph<bvh_type>                       graph_type;
      typedef typename graph_type::node_ptr_type       node_ptr_type;
      typedef typename graph_type::edge_ptr_type       edge_ptr_type;
      typedef typename graph_type::edge_iterator       edge_iterator;
      typedef typename graph_type::edge_ptr_iterator   edge_ptr_iterator;
      typedef typename graph_type::node_iterator       node_iterator;
      typedef typename graph_type::edge_ptr_container  edge_ptr_container;
      typedef typename graph_type::real_type           real_type;
      typedef typename bvh_type::volume_type           volume_type;
      typedef typename bvh_type::geometry_type         geometry_type;

    protected:

      graph_type *       m_graph;    ///< Pointer to graph that is currently being worked on.
      edge_ptr_container m_matched;  ///< binary matched edges, that are up for collapsing.

    public:

      /**
      * Initialization Method.
      * This method sets up internal data structures, needed
      *  for picking edges etc..
      *
      * @param graph    A reference to the graph that the policiy
      *                 is working on.
      */
      void init(graph_type & graph)
      {
        m_graph = &graph;
        match_edges();
      }

      /**
      * Update Node after Collapse.
      * This method is intendend to be called on the resulting
      * node after a collapse operation.
      *
      *
      * @param node    A pointer to the node.
      */
      void update(node_ptr_type /*node*/)
      {
        if(m_matched.empty())
          match_edges();
      }

      /**
      * Retrive next edge that should be collapsed.
      *
      * @return     A pointer to the graph edge that should be collapsed.
      */
      edge_ptr_type get_next_edge()
      {
        edge_ptr_type edge( m_matched.front() );
        m_matched.pop_front();
        return edge;
      }

      /**
      * More Edges to Collapse Query.
      *
      * @return    If the graph contains more edges that should be collapsed
      *            then the return value is true otherwise it is false.
      */
      const bool has_more_edges()const {   return (m_graph->size_edges()>0);  }

      /**
      * Create BV Decision Method.
      * This method is used by the Bottom-Up-Construction algorithm to determine
      * if an edge-collapse should result in a new BV in the BVH, or if more
      * sub-nodes are merged into the same region.
      *
      * Default implementation is based on the branching factor. See method degree().
      *
      * @param node    A pointer to the node, which is the result from a edge-collapse.
      *
      * @return        If a BV should be created then the return value is true
      *                otherwise it is false.
      */
      bool should_create_bv(node_ptr_type node)
      {
        bool possible_legal_collapse_exist = false;

        edge_iterator edge = node->edge_begin();
        edge_iterator end  = node->edge_end();
        for(;edge!=end;++edge)
        {
          node_ptr_type other = (edge->A() == node)?edge->B():edge->A();

          if(node->size_sub_nodes()+other->size_sub_nodes()<degree())
            possible_legal_collapse_exist = true;
        }
        if(!possible_legal_collapse_exist)
        {
          return true;
        }
        if(node->size_sub_nodes() >= degree())
          return true;
        if( (m_graph->size_edges()+1)< degree() )
          return true;
        return false;
      }

      /**
      * Volume Fitting Method.
      *
      * @param g0   An iterator to the first geometric primitive
      *             that should be covered by the fitted volume.
      * @param g1   An iterator to one position pass the last geometric
      *             primitive that should be covered by the fitted volume.
      * @param v0   An iterator to the first volume that should be
      *             covered by the fitted volume.
      * @param v1   An iterator to one position pass the last volume
      *             that should be covered by the fitted volume.
      *
      * @return     The fitted volume.
      */
      template<typename geometry_iterator,typename volume_iterator>
      volume_type fit(geometry_iterator g0,geometry_iterator g1,volume_iterator v0,volume_iterator v1){  return volume_type(); }

    protected:

      void match_edges()
      {
        if(m_graph->size_edges()==0)
          return;
        m_matched.clear();

        {
          edge_iterator edge = m_graph->edge_begin();
          edge_iterator end  = m_graph->edge_end();
          for(;edge != end; ++edge)
            edge->m_tag = 0;
        }
        {
          node_iterator node = m_graph->node_begin();
          node_iterator end  = m_graph->node_end();
          for(;node != end; ++node)
            node->m_tag2 = node->m_tag = 0;
        }

        std::list<node_ptr_type> Q;
        node_ptr_type first( m_graph->edge_begin()->A() );
        Q.push_back( first );

        while(!Q.empty())
        {
          node_ptr_type node( Q.front() );
          Q.pop_front();
          node->m_tag = 1;

          edge_ptr_iterator e    = node->edge_ptr_begin();
          edge_ptr_iterator end  = node->edge_ptr_end();
          for(;e!=end;++e)
          {
            if( (*e)->m_tag)
              continue;
            (*e)->m_tag = 1;
            node_ptr_type other = ( (*e)->A() == node)? (*e)->B(): (*e)->A();
            if(!other->m_tag)
            {
              Q.push_back(other);
            }
            //--- should we match node and other?
            if(!node->m_tag2 && !other->m_tag2)
            {
              node->m_tag2 = 1;
              other->m_tag2 = 1;
              edge_ptr_type match( *e );
              m_matched.push_back( match );
            }
          }
        }
      }

      /**
      * The degree (or branching factor) of BVH.
      * This method is used to decide the maximum branching factor
      * of the generated BVH.
      *
      * Note this is a virtual-method, this means it can be overriden by
      * inhertance, such that end-user can define a genric branching factor.
      *
      * See method should_create_bv(node_type*) for example of usage.
      *
      * @return        The branching factor.
      */
      const unsigned int degree() const { return 2; }
    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_BINARY_MATCHING_BOTTOM_UP_POLICY_H
#endif
