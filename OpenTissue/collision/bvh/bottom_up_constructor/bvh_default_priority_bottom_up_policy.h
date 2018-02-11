#ifndef OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_DEFAULT_PRIORITY_BOTTOM_UP_POLICY_H
#define OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_DEFAULT_PRIORITY_BOTTOM_UP_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_less_ptr.h>
#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_graph.h>
#include <OpenTissue/core/math/math_constants.h>


#include <cmath>       //Needed for fabs()

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * Default Priority Bottom Up Policy.
    * This class implements a default bottom-up consruction policy. The order edges are
    * collapsed in is based on increasing priority.
    *
    * A collapse results in an update of the priorities of all edges
    * topologically affected by the change in the graph.
    */
    template<typename bvh_type>
    class DefaultPriorityBottomUpPolicy
    {
    public:

      typedef BVHGraph<bvh_type>                       graph_type;
      typedef typename graph_type::node_ptr_type       node_ptr_type;
      typedef typename graph_type::edge_ptr_type       edge_ptr_type;
      typedef typename graph_type::edge_iterator       edge_iterator;
      typedef typename graph_type::edge_ptr_iterator   edge_ptr_iterator;
      typedef typename graph_type::real_type           real_type;
      typedef typename bvh_type::volume_type           volume_type;
      typedef typename bvh_type::geometry_type         geometry_type;

    protected:

      graph_type * m_graph;  ///< Pointer to graph that is currently being worked on.

    public:
      
      virtual ~DefaultPriorityBottomUpPolicy(){};
      
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
        edge_ptr_iterator edge = m_graph->edge_ptr_begin();
        edge_ptr_iterator end  = m_graph->edge_ptr_end();
        for(;edge != end; ++edge)
        {
          edge_ptr_type ptr( *edge );
          ptr->priority() = priority(ptr);
        }
        update_heap();
      }

      /**
      * Update Node after Collapse.
      * This method is intendend to be called on the resulting
      * node after a collapse operation.
      *
      *
      * @param node    A pointer to the node.
      */
      void update(node_ptr_type node)
      {
        edge_ptr_iterator edge = node->edge_ptr_begin();
        edge_ptr_iterator end  = node->edge_ptr_end();
        for(; edge!=end; ++edge)
        {
          edge_ptr_type ptr( *edge );
          ptr->priority() = priority(ptr);
        }
        update_heap();
      }

      /**
      * Retrive next edge that should be collapsed.
      *
      * @return     A pointer to the graph edge that should be collapsed.
      */
      edge_ptr_type  get_next_edge() const{   return edge_ptr_type( *(m_graph->edge_ptr_begin()) ); }

      /**
      * More Edges to Collapse Query.
      *
      * @return    If the graph contains more edges that should be collapsed
      *            then the return value is true otherwise it is false.
      */
      const bool has_more_edges() const {   return (m_graph->size_edges()>0);  }

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
      bool should_create_bv(node_ptr_type node){  return (  node->size_sub_nodes() >= degree()   ||  (m_graph->size_edges()+1)< degree() ); }

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

      /**
      * Update Heap.
      * Internally used method, called whenever edge priorities
      * are changed to re-inforce heap-property.
      *
      * See method update(node_type*) and init(graph_type&) for examples of usage.
      */
      void update_heap()
      {
        //--- KE 12-10-2004: STL heap stuff does not work on list-types, unfortuantely
        //--- graph edges are stored in a list:-(
        //---
        //---    std::make_heap(m_graph->m_edges.begin(),m_graph->m_edges.end(),OpenTissue::utility::less_ptr<edge_type *>());
        //---    std::sort_heap(m_graph->m_edges.begin(),m_graph->m_edges.end(),OpenTissue::utility::less_ptr<edge_type *>());
        //---
        //--- Our solution is simply to apply a brute force sorting, it is
        //--- not very efficient, but it will get the job done.

        //std::sort(m_graph->edge_ptr_begin(), m_graph->edge_ptr_end(), OpenTissue::utility::less_ptr<edge_ptr_type>());   // TODO can not determine difference_type on iterator????
        m_graph->m_edges.sort(OpenTissue::utility::less_ptr<edge_ptr_type>());

      }

      /**
      * Edge Priority Computation.
      * This method is used to assign priorities to edges in the graph.
      *
      * Note this is a virtual-method, this means it can be overriden by
      * inhertance, such that end-user can define genric priorty strategies.
      *
      * Default stategy, tries to favor a balanced BVH.
      *
      * See method update(node_type*) and init(graph_type&) for examples of usage.
      *
      * @param edge   A pointer to the edge, from which a priority should be computed.
      *
      * @return       The priority of the specified edge.
      */
      virtual real_type priority(edge_ptr_type edge)
      {
        using std::fabs;

        real_type a = edge->A()->size_subtree();
        real_type b = edge->B()->size_subtree();
        if((edge->A()->size_edges() + edge->B()->size_edges()) == degree())
        {
          return 0;
        }
        if((edge->A()->size_edges() + edge->B()->size_edges()) > degree())
        {
          return math::detail::highest<real_type>();
        }
        return (a + b + fabs(a - b ));
      }

      /**
      * The degree (or branching factor) of BVH.
      * This method is used to decide the maximum branching factor
      * of the generated BVH.
      *
      * Note this is a virtual-method, this means it can be overriden by
      * inhertance, such that end-user can define a genric branching factor.
      *
      * See method create_bv(node_type*) for example of usage.
      *
      * @return        The branching factor.
      */
      virtual const unsigned int degree() const { return 2; }

    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_DEFAULT_PRIORITY_BOTTOM_UP_POLICY_H
#endif
