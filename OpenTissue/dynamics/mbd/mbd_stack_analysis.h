#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_ANALYSIS_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_ANALYSIS_H
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
  namespace mbd
  {
    /**
    * Stack Analysis.
    *
    * This auxiliary tool analyses a contact group in order to see if it forms a ``stack''.
    *
    * A stack is loosely defined as when something is in contact with a fixed body.
    *
    * Contacts (and constraints) can be assigned a stack layer, equivalent to
    * how long a path there is to the closest fixed body.
    *
    * The two incident bodies at a contact can be assigned a lower or upper state. Depending
    * on which one that is closest to a fixed body. This state can be deduced from the stack
    * height of the bodies. Stack height is the smallest number of contacts to a fixed body.
    *
    */
    template<typename mbd_types>
    class StackAnalysis
    {
    protected:

      typedef typename mbd_types::math_policy::index_type    index_type;
      typedef typename mbd_types::math_policy::index_type    size_type;

      typedef typename mbd_types::math_policy::real_type     real_type;
      typedef typename mbd_types::math_policy::vector3_type  vector3_type;
      typedef typename mbd_types::group_type                 group_type;
      typedef typename mbd_types::body_type                  body_type;
      typedef typename mbd_types::edge_type                  edge_type;

      typedef typename mbd_types::contact_type              contact_type;
      typedef typename mbd_types::material_type             material_type;
      typedef typename mbd_types::edge_ptr_container        edge_ptr_container;
      typedef typename mbd_types::indirect_edge_iterator             indirect_edge_iterator;
      typedef typename mbd_types::group_container           group_container;

    protected:

      typedef typename std::vector<contact_type*>             contact_ptr_heap;
      typedef typename std::list<body_type*>                  body_ptr_queue;

    public:

      class node_traits
      {
      public:
        size_type m_sa_stack_height;   ///< The number of bodies way from closest fixed body. Fixed bodies have height zero.
        bool      m_sa_queue_tag;      ///< Boolean flag set to true if node when node have been  pushed into queue.

        node_traits()
          : m_sa_stack_height(0) 
        {}

      };

      class edge_traits
      {
      public:
        size_type m_sa_stack_layer;       ///< Equal to the minimum stack height of incident nodes.
        size_type m_sa_visit_time_stamp;  ///< Time stamp indicating last time the edge was traversed.

        edge_traits() 
          : m_sa_visit_time_stamp(0) 
        {}
      };

      class constraint_traits{ };

    protected:

      index_type  m_time_stamp;         ///< Time-stamp of last time algorithm was invoked.

    public:

      StackAnalysis() 
        : m_time_stamp(0) 
      {}

    public:

      /**
      * Analyse Contact group_type.
      *
      * @param group    The contact group that should be analysed.
      * @param layers   Upon return this argument holds the stack layers, they are stored in increasing height.
      *
      * @return         The number of stack layers, this is equal to the maximum height of the stack.
      */
      size_type analyze(group_type & group,group_container & layers)
      {
        using std::min;
        using std::max;

        layers.clear();

        size_type N = group.size_bodies();
        if(N==0)
          return 0;

        ++m_time_stamp;
        body_ptr_queue Q;
        {
          typename group_type::indirect_body_iterator begin = group.body_begin();
          typename group_type::indirect_body_iterator end = group.body_end();
          typename group_type::indirect_body_iterator body;
          for( body=begin; body!=end; ++body )
          {
            if(body->is_fixed() || body->is_scripted())
            {
              body->m_sa_stack_height = 0;
              Q.push_back(&(*body));
              body->m_sa_queue_tag = true;
            }
            else
            {
              body->m_sa_stack_height = N;
              body->m_sa_queue_tag = false;
            }
          }
        }
        if(Q.empty() || (group.size_contacts()==0))
        {
          //--- No fixed objects, just return the original contact group as result...
          layers.resize(1);
          layers[0] = group;
          return 1;
        }

        edge_ptr_container edges;  //--- Used to keep pointers to all edges in contact group
        size_type height = 0; //--- Used to figure out how many layers there is in the stack
        while(!Q.empty())
        {
          body_type * body = Q.front();
          Q.pop_front();
          typename body_type::indirect_edge_iterator begin = body->edge_begin();
          typename body_type::indirect_edge_iterator end = body->edge_end();
          for(typename body_type::indirect_edge_iterator edge = begin;edge!=end;++edge)
          {
            if(!edge->is_up_to_date())//--- Make sure we only process edges that contain valid cached information.
              continue;

            body_type * next = (body==edge->get_body_A())? edge->get_body_B(): edge->get_body_A();
            bool has_joint = body->has_joint_to(next);
            if(edge->size_contacts()==0 && !has_joint)//--- Make usre we only process edges that contain contact information
              continue;

            if(!next->m_sa_queue_tag)
            {
              Q.push_back(next);
              next->m_sa_queue_tag = true;
            }

            next->m_sa_stack_height = min(next->m_sa_stack_height,body->m_sa_stack_height + 1);

            if(next->m_sa_stack_height == body->m_sa_stack_height && next->m_sa_stack_height!=0)
              edge->m_sa_stack_layer = next->m_sa_stack_height-1;
            else
              edge->m_sa_stack_layer = min(next->m_sa_stack_height,body->m_sa_stack_height);

            height = max(height,edge->m_sa_stack_layer);

            if(edge->m_sa_visit_time_stamp!=m_time_stamp)
            {
              edge->m_sa_visit_time_stamp=m_time_stamp;
              edges.push_back(&(*edge));
            }
          }
        }
        if(edges.empty())
        {
          //--- This means that the fixed objects were not in contact with any non-fixed objects...
          layers.resize(1);
          layers[0] = group;
          return 1;
        }
        //--- Now simply traverse edges and build stack layers
        {
          layers.resize(height+1);
          indirect_edge_iterator begin = edges.begin();
          indirect_edge_iterator end = edges.end();
          for(indirect_edge_iterator edge = begin;edge!=end;++edge)
          {
            size_type layer = edge->m_sa_stack_layer;

            for(typename edge_type::contact_iterator contact = edge->contact_begin();contact!=edge->contact_end();++contact)
            {
              layers[layer].m_contacts.push_back( &(*contact) );
            }

            if(edge->get_body_A()->has_joint_to(edge->get_body_B()))
            {
              for(typename body_type::indirect_joint_iterator joint = edge->get_body_A()->joint_begin();joint!=edge->get_body_A()->joint_end();++joint)
              {
                if(joint->get_socket_A()->get_body()==edge->get_body_B() || joint->get_socket_B()->get_body()==edge->get_body_B())
                {
                  layers[layer].m_constraints.push_back(&(*joint));
                }
              }
            }
          }
        }
        //--- Now traverse bodies and add them to the layers
        {
          typename group_type::indirect_body_iterator begin = group.body_begin();
          typename group_type::indirect_body_iterator end = group.body_end();
          for(typename group_type::indirect_body_iterator body=begin;body!=end;++body)
          {
            if(body->m_sa_stack_height==N)
            {
              //--- somehow this is a non-fixed body that was not in contact with anything else, we simply add it to top layer...
              layers[height].m_bodies.push_back(&(*body));
              continue;
            }
            bool in_lower = false;
            bool in_upper = false;
            typename body_type::indirect_edge_iterator begin = body->edge_begin();
            typename body_type::indirect_edge_iterator end = body->edge_end();
            for(typename body_type::indirect_edge_iterator edge = begin;edge!=end;++edge)
            {
              if(!edge->is_up_to_date())//--- Make sure we only process edges that contain valid cached information.
                continue;
              body_type * other = (edge->get_body_A()==&(*body))?edge->get_body_B():edge->get_body_A();
              if(other->m_sa_stack_height==N)//--- Bodies are not in contact, so ignore this edge
                continue;
              if(other->m_sa_stack_height > body->m_sa_stack_height)
                in_upper = true;
              if(other->m_sa_stack_height < body->m_sa_stack_height)
                in_lower = true;
              if(in_upper && in_lower)
                break;
            }
            if(in_upper)
              layers[body->m_sa_stack_height].m_bodies.push_back(&(*body));
            if(in_lower)
              layers[body->m_sa_stack_height - 1].m_bodies.push_back(&(*body));
          }
        }
        return height+1;
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_ANALYSIS_H
#endif
