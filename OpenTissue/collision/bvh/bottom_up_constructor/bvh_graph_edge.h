#ifndef OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_EDGE_H
#define OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_EDGE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_graph_node.h>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * A BVH Graph Edge.
    */
    template <typename bvh_type>
    class BVHGraphEdge
    {
    public:

      friend class BVHGraph<bvh_type>;

    public:

      //--- Conenience stuff for better readability
      typedef BVHGraph<bvh_type>                   graph_type;
      typedef typename graph_type::node_ptr_type   node_ptr_type;
      typedef typename graph_type::edge_type       edge_type;
      typedef typename graph_type::real_type       real_type;

    protected:

      node_ptr_type  m_A;         ///< Pointer to incidient graph node.
      node_ptr_type  m_B;         ///< Pointer to other incidient graph node.
      real_type      m_priority;  ///< Edge priority, used for picking edges, when an edge
      ///< is collapsed, the two incident nodes are merged
      ///< into one new graph node.
      ///< If the merge do not result in a new BV node, then the
      ///< union of the sub nodes of A and B are stored as sub
      ///< nodes in the new graph node. If a new BV node is created
      ///< as a result of the merge then the new graph node will not
      ///< have any sub-nodes.
    public:

      unsigned int m_tag;                   ///< Tag, can be used for traversing graph structure.

    public:

      BVHGraphEdge()
        : m_A()
        , m_B()
        , m_priority(0) 
      {}

    public:

      node_ptr_type const & A() const {  return m_A; }
      node_ptr_type const & B() const {  return m_B; }
      node_ptr_type & A() {  return m_A; }
      node_ptr_type & B() {  return m_B; }
      const real_type & priority() const {  return m_priority; }
      real_type & priority() {  return m_priority; }
      const bool operator>(const edge_type & edge) const {  return (this->m_priority>edge.m_priority);}
      const bool operator<(const edge_type & edge) const {  return (this->m_priority<edge.m_priority);}

    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_EDGE_H
#endif
