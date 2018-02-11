#ifndef OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_H
#define OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_graph_node.h>
#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_graph_edge.h>

#include <boost/shared_ptr.hpp>
#include <boost/iterator/indirect_iterator.hpp>

#include <cmath>       //Needed for fabs()
#include <list>        //Needed for graph data structure: edges, nodes and volumes
#include <iostream>    //Needed for debug output. NOTE: we should consider implementing a logging fascility!

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * A BVH Graph.
    * This data structure is used exclusively for bottom-up construction of BVHs.
    */
    template <typename bvh_type>
    class BVHGraph
    {
    public:

      //--- Conenience stuff for better readability
      typedef BVHGraph<bvh_type>                     graph_type;
      typedef BVHGraphNode<bvh_type>                 node_type;
      typedef BVHGraphEdge<bvh_type>                 edge_type;
      //typedef node_type *                            node_ptr_type;
      //typedef edge_type *                            edge_ptr_type;
      typedef boost::shared_ptr<node_type>           node_ptr_type;
      typedef boost::shared_ptr<edge_type>           edge_ptr_type;
      typedef boost::shared_ptr<node_type const>     const_node_ptr_type;
      typedef boost::shared_ptr<edge_type const>     const_edge_ptr_type;
      typedef typename bvh_type::geometry_container  geometry_container;
      typedef typename bvh_type::geometry_type       geometry_type;
      typedef typename bvh_type::volume_type         volume_type;
      typedef typename std::list<volume_type>        volume_container;
      typedef float                                  real_type;

    public:

      typedef typename std::list< edge_ptr_type >                           edge_ptr_container;
      typedef typename edge_ptr_container::iterator                         edge_ptr_iterator;
      typedef typename edge_ptr_container::const_iterator                   const_edge_ptr_iterator;
      typedef boost::indirect_iterator<edge_ptr_iterator,edge_type>         edge_iterator;
      typedef boost::indirect_iterator<const_edge_ptr_iterator,edge_type>   const_edge_iterator;

      edge_iterator       edge_begin()       { return edge_iterator(m_edges.begin());       }
      edge_iterator       edge_end()         { return edge_iterator(m_edges.end());         }
      const_edge_iterator edge_begin() const { return const_edge_iterator(m_edges.begin()); }
      const_edge_iterator edge_end()   const { return const_edge_iterator(m_edges.end());   }

      edge_ptr_iterator       edge_ptr_begin()       { return m_edges.begin();       }
      edge_ptr_iterator       edge_ptr_end()         { return m_edges.end();         }
      const_edge_ptr_iterator edge_ptr_begin() const { return m_edges.begin();       }
      const_edge_ptr_iterator edge_ptr_end()   const { return m_edges.end();         }

      typedef typename std::list< node_ptr_type >                           node_ptr_container;
      typedef typename node_ptr_container::iterator                         node_ptr_iterator;
      typedef typename node_ptr_container::const_iterator                   const_node_ptr_iterator;
      typedef boost::indirect_iterator<node_ptr_iterator,node_type>         node_iterator;
      typedef boost::indirect_iterator<const_node_ptr_iterator,node_type>   const_node_iterator;

      node_iterator node_begin()             { return node_iterator(m_nodes.begin());       }
      node_iterator node_end()               { return node_iterator(m_nodes.end());         }
      const_node_iterator node_begin() const { return const_node_iterator(m_nodes.begin()); }
      const_node_iterator node_end()   const { return const_node_iterator(m_nodes.end());   }

      node_ptr_iterator node_ptr_begin()             { return m_nodes.begin();       }
      node_ptr_iterator node_ptr_end()               { return m_nodes.end();         }
      const_node_ptr_iterator node_ptr_begin() const { return m_nodes.begin();       }
      const_node_ptr_iterator node_ptr_end()   const { return m_nodes.end();         }

    public: // TODO should be protected. However we got a little problem with priority bottom up policy

      edge_ptr_container  m_edges;    ///< All edges in graph.

    protected:
      node_ptr_container  m_nodes;    ///< All nodes in graph.

    public:

      BVHGraph(){}

      ~BVHGraph() { clear(); }

    public:

      /**
      * Retrive the number of edges in graph.
      *
      * @return          Number of edges.
      */
      std::size_t size_edges() const {return m_edges.size();}

      /**
      * Retrive the number of nodes in graph.
      *
      * @return          Number of nodes.
      */
      std::size_t size_nodes()const{return m_nodes.size();}

      /**
      * Insert Node.
      *
      * @param geometry  The initial geometry covered by the new node.
      *
      * @return          A pointer to the new node.
      */
      node_ptr_type insert(geometry_type const & g){ return insert(volume_type(),g); }

      /**
      * Insert Node.
      *
      * @param volume    The initial volume of the new node.
      *
      * @return          A pointer to the new node.
      */
      node_ptr_type insert(volume_type const & volume)
      {
        geometry_container G;
        return insert(volume,G);
      }

      /**
      * Insert Node.
      *
      * @param volume    The initial volume of the new node.
      * @param geometry  The initial geometry covered by the new node.
      *
      * @return          A pointer to the new node.
      */
      node_ptr_type insert(volume_type const & volume,geometry_type const & g)
      {
        geometry_container G;
        G.push_back(g);
        return insert(volume,G);
      }

      /**
      * Insert Node.
      *
      * @param volume    The initial volume of the new node.
      * @param geometry  The initial geometry covered by the new node.
      *
      * @return          A pointer to the new node.
      */
      node_ptr_type insert(volume_type const & volume,geometry_container & geometry)
      {
        node_ptr_type node( new node_type() );
        node->m_volume = volume;
        if(!geometry.empty())
        {
          //--- KE 13-10-2004: Something wrong copy does not work???
          //std::copy(geometry.begin(),geometry.end(),node->m_coverage.begin());
          node->m_coverage.insert(node->m_coverage.end(),geometry.begin(),geometry.end());
        }
        m_nodes.push_back(node);
        return node;
      }

      /**
      * Insert Edge.
      * This method inserts and edge between the two specified nodes.
      *
      * The method guards against self-loofs and redudant edges, both
      * of which is illegal.
      *
      * @param A   A pointer to the first node.
      * @param B   A pointer to the second node.
      *
      * @return    A pointer to the newly created edge.
      */
      edge_ptr_type insert(node_ptr_type A,node_ptr_type B)
      {
        assert(A!=B);
        //--- Test if edge already exist
        edge_ptr_iterator e    = A->edge_ptr_begin();
        edge_ptr_iterator end  = A->edge_ptr_end();
        for(;e!=end;++e)
        {
          if(( (*e)->A()==A && (*e)->B()==B)||( (*e)->A()==B && (*e)->B()==A))
            return edge_ptr_type(*e);
        }
        edge_ptr_type new_edge(new edge_type());
        new_edge->m_A = A ;
        A->m_edges.push_back(new_edge);
        new_edge->m_B = B;
        B->m_edges.push_back(new_edge);
        m_edges.push_back(new_edge);
        return new_edge;
      }

      /**
      * Clear Graph.
      * Removes all edges and nodes.
      */
      void clear()
      {
        m_nodes.clear();
        m_edges.clear();
      }

      /**
      * Edge Collapse.
      *
      * @param edge   A pointer to the edge that should be collapsed into a node.
      *
      * @return       A pointer to the new node that corresponds to the collapsed edge. The previous
      *               incident nodes are now sub-nodes of the new node.
      */
      node_ptr_type collapse(edge_ptr_type edge)
      {
        node_ptr_type C( new node_type() );
        node_ptr_type A = edge->A();
        node_ptr_type B = edge->B();
        //--- Move all edges to new node C
        C->m_edges.splice(C->m_edges.end(),A->m_edges);
        C->m_edges.splice(C->m_edges.end(),B->m_edges);
        assert(A->m_edges.empty());
        assert(B->m_edges.empty());
        //--- Re-assign node pointers going to A or B
        {
          edge_iterator e    = C->edge_begin();
          edge_iterator end  = C->edge_end();
          for(; e != end; ++e)
          {
            if(e->A() == A)
              e->A() = C;
            if(e->B() == A)
              e->B() = C;
            if(e->A() == B)
              e->A() = C;
            if(e->B()== B)
              e->B() = C;
          }
        }
        //--- The move might have caused some inconsistency, so
        //--- we need to clean up things, removing self-loops and
        //--- multiple defined edges.

        for(edge_ptr_iterator e = C->edge_ptr_begin(); e != C->edge_ptr_end(); )
        {
          if( (*e)->A() == (*e)->B() )//--- self loop test
          {
            edge_ptr_type self_loop( *e ); 
            remove(self_loop);
            e = C->edge_ptr_begin();//--- deletion have invalidated iterator...this is a simple workaround
          }
          else
            ++e;
        }

        for(edge_ptr_iterator i=C->edge_ptr_begin();i!=C->edge_ptr_end();++i)
        {
          edge_ptr_iterator j = i;
          ++j;
          for(;j!=C->edge_ptr_end();)
          {
            edge_ptr_iterator k = j;
            ++j;
            if(
              ((*k)->A()==(*i)->A() && (*k)->B()==(*i)->B())
              ||
              ((*k)->A()==(*i)->B() && (*k)->B()==(*i)->A())
              )//--- Redundant edge test
            {
              edge_ptr_type redundant( *k );
              remove(redundant);
            }
          }
        }
        //--- Take care of coverage geometry if any...
        //C->m_coverage.insert(C->m_coverage.end(),A->m_coverage.begin(),A->m_coverage.end());
        //A->m_coverage.clear();
        //C->m_coverage.insert(C->m_coverage.end(),B->m_coverage.begin(),B->m_coverage.end());
        //B->m_coverage.clear();
        C->m_coverage.splice(C->m_coverage.end(),A->m_coverage);
        C->m_coverage.splice(C->m_coverage.end(),B->m_coverage);

        //---  Initialize internal data in the new node
        C->m_subtree_size = A->m_subtree_size + B->m_subtree_size;
        if(!A->size_sub_nodes())
          C->m_sub_nodes.push_back(A);
        else
        {
          C->m_sub_nodes.splice(C->m_sub_nodes.end(),A->m_sub_nodes);
          remove(A);
        }
        if(!B->size_sub_nodes())
          C->m_sub_nodes.push_back(B);
        else
        {
          C->m_sub_nodes.splice(C->m_sub_nodes.end(),B->m_sub_nodes);
          remove(B);
        }
        C->m_height = C->max_sub_node_height();
        m_nodes.push_back(C);
        return C;
      }



      /**
      * Remove Subnodes.
      * This method is invoked by the BottomUpConstructor algorithm.
      *
      * @param node   A pointer to the node where all subnode should be removed.
      */
      void remove_sub_nodes(node_ptr_type node)
      {
        if(!node->size_sub_nodes())
          return;

        node_ptr_iterator sub_node = node->sub_node_ptr_begin();
        node_ptr_iterator end      = node->sub_node_ptr_end();
        for(;sub_node!=end;++sub_node)
        {
          node_ptr_type target( *sub_node );
          m_nodes.remove( target );
        }
        node->m_sub_nodes.clear();
      }

    protected:

      /**
      * Remove Edge.
      * This method is invoked by the collapse method.
      *
      * @param edge   A poiniter to the edge that should be removed.
      */
      void remove(edge_ptr_type edge)
      {
        assert(edge);
        assert(edge->A());
        assert(edge->B());

        edge->m_A->m_edges.remove(edge);
        edge->m_B->m_edges.remove(edge);

        m_edges.remove(edge);//--- KE 11-10-2004: Preformance could be improved by using iterators

        edge.reset();
      }

      /**
      * Remove Node.
      * This method is invoked by the remove_subnodes() method.
      *
      * @param node   A pointer to the node that should be removed.
      */
      void remove(node_ptr_type node)
      {
        assert(!node->size_sub_nodes() || !"BVHGraph::remove(node_ptr_type): Node has sub nodes!");
        assert(!node->size_edges()     || !"BVHGraph::remove(node_ptr_type): Node got incident edges");
        m_nodes.remove(node);

        node.reset();
      }

    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_GRAPH_H
#endif
