#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_CORE_ACCESS_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_CORE_ACCESS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace polymesh
  {
    class polymesh_core_access
    {
    public:

      template<typename feature_iterator,typename handle>
      static void set_self_handle(feature_iterator feature,handle self) { feature->set_handle(self); }

      template<typename feature_iterator,typename mesh_type>
      static void set_owner(feature_iterator feature,mesh_type * owner){ feature->set_owner(owner); }

      template<typename vertex_iterator,typename halfedge_handle>
      static void set_outgoing_halfedge_handle(vertex_iterator v,halfedge_handle h)
      {
        v->set_outgoing_halfedge_handle(h);
      }

      template<typename halfedge_iterator,typename halfedge_handle>
      static void set_next_handle(halfedge_iterator h,halfedge_handle n)
      {
        h->set_next_handle(n);
      }

      template<typename halfedge_iterator,typename face_handle>
      static void set_face_handle(halfedge_iterator h,face_handle f)
      {
        h->set_face_handle(f);
      }

      template<typename halfedge_iterator,typename halfedge_handle>
      static void set_twin_handle(halfedge_iterator h,halfedge_handle t)
      {
        h->set_twin_handle(t);
      }

      template<typename halfedge_iterator,typename vertex_handle>
      static void set_destination_handle(halfedge_iterator h,vertex_handle v)
      {
        h->set_destination_handle(v);
      }

      template<typename face_iterator,typename halfedge_handle>
      static void set_border_halfedge_handle(face_iterator f,halfedge_handle h)
      {
        f->set_border_halfedge_handle(h);
      }

      template<typename halfedge_iterator,typename edge_handle>
      static void set_edge_handle(halfedge_iterator h,edge_handle e){ h->set_edge_handle(e); }

      template<typename edge_iterator,typename halfedge_handle>
      static void set_halfedge0_handle(edge_iterator e,halfedge_handle h){ e->set_halfedge0_handle(h); }

      template<typename edge_iterator,typename halfedge_handle>
      static void set_halfedge1_handle(edge_iterator e,halfedge_handle h){ e->set_halfedge1_handle(h); }



      /**
      * This method makes sure that the outgoing halfedge from a vertex
      * is a boundary half-edge (i.e. it has no incident face) if one exist.
      */
      template<typename vertex_iterator>
      static void adjust_outgoing_halfedge_handle(vertex_iterator v)
      {
        typedef typename vertex_iterator::value_type             vertex_type;
        typedef typename vertex_type::mesh_type                  mesh_type;
        typedef typename mesh_type::vertex_halfedge_circulator   vertex_halfedge_circulator;

        vertex_halfedge_circulator  circulator( *v ), end;
        for(;circulator!=end;++circulator)
        {
          if( circulator->get_face_handle().is_null())
          {
            set_outgoing_halfedge_handle(v,circulator->get_handle());
            return;
          }
        }
      }

      /**
      * Unlinks a halfedge (and its twin) from its destination vertex.
      *
      * @param h   The halfedge to be unlinked.
      *
      * @return    If the halfedge was unlinked then the return value is true, otherwise it is false.
      */
      template<typename halfedge_iterator>
      bool unlink(halfedge_iterator h)
      {
        typedef typename halfedge_iterator::value_type  halfedge_type;
        typedef typename halfedge_type::mesh_type       mesh_type;
        //typedef typename mesh_type::halfedge_iterator   halfedge_iterator;
        typedef typename mesh_type::halfedge_iterator   vertex_iterator;
        //                                    //
        //  \     h1_prev                     //
        //  _\|                               //
        //    \                               //
        //     \                              //
        //      \          h1                 //
        //        v   -------->----------     //
        //        *                           //
        //       /    --------<----------     //
        //      /          h0 (h)             //
        //    |/_                             //
        //    /                               //
        //        h0_next                     //
        //                                    //
        if(h.get_destination_handle().is_null())
        {
          assert(!"unlink(h): Illegal topology!");
          return false;
        }
        mesh_type * owner = h.get_owner();
        if(!owner)
        {
          assert(!"unlink(h): No owner mesh!");
          return false;
        }
        //--- Get iterators for all
        halfedge_iterator h1      = h.get_twin_iterator();
        halfedge_iterator h0      = h1->get_twin_itreator();
        vertex_iterator   v       = h.get_destination_iterator();
        halfedge_iterator h1_prev = h1->get_prev_iterator();
        halfedge_iterator h0_next = h0->get_next_iterator();
        //--- more than one incident edge to vertex
        if(h1_prev->m_self != h0->m_self)
        {
          h1_prev->m_next   = h0_next->m_self;
          h0_next->m_prev   = h1_prev->m_self;
        }
        //--- exactly one incident edge (the one we are aboud to unlink) to vertex
        if(h1_prev->m_self == h0->m_self)
        {
          v->m_outgoing_halfedge = owner->null_halfedge_handle();
        }
        //--- more than one incident edge to vertex, but we are removing the edge v is pointing to.
        if(h1->m_self == v->m_outgoing_halfedge)
        {
          v->m_outgoing_halfedge = h0_next->m_self;
        }
        h0->m_next        = owner->null_halfedge_handle();
        h0->m_destination = owner->null_vertex_handle();
        h1->m_prev        = owner->null_halfedge_handle();
        adjust_outgoing_halfedge_handle(v);
        return true;
      }

      /**
      * Links halfedge into v's topology, such that h points to v.
      *
      * @param h0   The halfedge that should have v as its new destination.
      * @param v    The vertex, must have an empty gap.
      *
      * @return     If succesfully linked the return value is true otherwise it is false.
      */
      template<typename halfedge_iterator,typename vertex_iterator>
      bool link(halfedge_iterator h0,vertex_iterator v)
      {
        //typedef typename halfedge_iterator::value_type  halfedge_type;
        //typedef typename halfedge_type::mesh_type       mesh_type:
        //typedef typename mesh_type::halfedge_iterator halfedge_iterator;
        //typedef typename mesh_type::halfedge_iterator vertex_iterator;
        //                                    //
        //  \     h1_prev                     //
        //  _\|                               //
        //    \                               //
        //     \                              //
        //      \          h1                 //
        //        v   -------->----------     //
        //        *                           //
        //       /    --------<----------     //
        //      /          h0 (h)             //
        //    |/_                             //
        //    /                               //
        //        h0_next                     //
        //                                    //
        if(!h0->get_destination_handle().is_null())
        {
          assert(!"link(h,v): Illegal topology, h allready had a destination!");
          return false;
        };
        halfedge_iterator h1 = h0->get_twin_iterator();
        //--- vertex is isolated
        if(v->m_outgoing_halfedge.is_null())
        {
          v->m_outgoing_halfedge = h1->m_self;
          h0->m_destination = v->m_self;
          h0->m_next = h1->m_self;
          h1->m_prev = h0->m_self;
          return true;
        }
        //--- Test for empty gap in vertex one-ring neighborhood
        halfedge_iterator h0_next = v->get_outgoing_halfedge_iterator();
        if(!h0_next->get_face_handle().is_null())
        {
          assert(!"link(h,v): vertex v did not have an empty gap");
          return false;
        }
        halfedge_iterator h1_prev = h0_next->get_prev_iterator();
        if(!h1_prev->get_face_handle().is_null())
        {
          assert(!"link(h,v): vertex v did not have an empty gap");
          return false;
        }

        //--- Now link everything together
        h0->m_next        = h0_next->m_self;
        h0_next->m_prev   = h0->m_self;
        h1_prev->m_next   = h1->m_self;
        h1->m_prev        = h1_prev->m_self;
        h0->m_destination =  v->m_self;

        adjust_outgong_edge(v);

        return true;
      }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_CORE_ACCESS_H
#endif
