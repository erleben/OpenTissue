#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_CACHING_CONTACT_GRAPH_ANALYSIS_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_CACHING_CONTACT_GRAPH_ANALYSIS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Caching Contact Graph Analysis.
    * This is a particular implementation of spatial-temporal-coherence analysis
    * algorithm to be used as a policy in the CollisionDetection class.
    *
    * The main idea is to store information in the contact graph edges (i.e. body
    * pairs) and then in each query test this information to see whether information
    * from last query can be reused, or if a quick-rejection is possible. Also a
    * connected component search is done on the contact graph in order to determine
    * independent contact groups.
    */
    template<typename types>
    class CachingContactGraphAnalysis
    {
    protected:

      typedef typename types::math_policy::index_type       size_type;
      typedef typename types::math_policy::vector3_type     vector3_type;
      typedef typename types::math_policy::matrix3x3_type   matrix3x3_type;
      typedef typename types::math_policy::quaternion_type  quaternion_type;
      typedef typename types::math_policy::coordsys_type    coordsys_type;

      typedef typename types::math_policy::real_type real_type;
      typedef typename types::configuration_type configuration_type;
      typedef typename types::group_container group_container;
      typedef typename types::group_ptr_container group_ptr_container;
      typedef typename types::group_type group_type;
      typedef typename types::edge_ptr_container edge_ptr_container;
      typedef typename types::indirect_edge_iterator indirect_edge_iterator;
      typedef typename types::body_type body_type;
      typedef typename types::contact_type contact_type;
      typedef typename types::edge_type edge_type;
      typedef typename types::joint_type  joint_type;

    public:

      class node_traits
      {
      public:
        node_traits()
          : m_ccg_absolute_resting(false)
          , m_ccg_r_prev(0,0,0)
          , m_ccg_Q_prev(0,1,0,0)
        {}

      public:

        vector3_type     m_ccg_r_prev;     ///< The previous (from last invocation of collision detection engine) position of the node in WCS.
        quaternion_type  m_ccg_Q_prev;     ///< The previous (from last invocation of collision detection engine) orientation of the node in WCS.
        bool m_ccg_absolute_resting;       ///< Boolean value indicating whatever this
        ///< body have been moved since the last invocation
        ///< of the collision detection engine. This
        ///< flag can be used to determine whatever
        ///< caching could be exploited.
      };
      class edge_traits
      {
      public:

        typedef enum{
          undefined,        ///< State of body pair is not yet defined.
          separating,       ///< body_type pair is separated.
          touching,         ///< body_type pair is touching (but not penetrating).
          penetrating       ///< body_type pair is penetrating.
        } CCG_StateType;        ///< Type for the state of the body pair.

        typedef enum{ white, grey, black } CCG_ColorType;

      public:
        edge_traits()
          : m_ccg_time_stamp(0) 
        {}
      public:
        coordsys_type m_ccg_xformAtoB;          ///< Storage holder for relative transform between A and B.
        coordsys_type m_ccg_xformBtoA;          ///< Storage holder for relative transform between B and A.
        coordsys_type m_ccg_xformAtoB_prev;     ///< Storage holder for the relative transform between A and B in the previous iteration of the contact analysis.
        size_type  m_ccg_time_stamp;         ///< Time-stamp indicating when the edge was detected by the broad phase collision detection algorithm.
        CCG_StateType m_ccg_state;              ///< The current contact state
        CCG_ColorType m_ccg_color;              ///< Color type is used for traversing contact groups.
      };
      class constraint_traits { };

    protected:

      configuration_type * m_configuration;     ///< A pointer to the configuration that holds the current configuration.
      size_type m_time_stamp;                ///< Time-stamp indicating which iteration the STC analysis i performed in.

    public:

      CachingContactGraphAnalysis(void):m_configuration(0),m_time_stamp(0){};

    public:

      /**
      * Post Broad Phase Analysis.
      *
      * @param edges     The overlaps reported from the broad phase collision detection phase.
      * @return          A boolean value indicating wheter a penetration have been detected at this stage in analysis.
      */
      const bool post_broad_phase_analysis( edge_ptr_container & edges )
      {
        ++m_time_stamp;
        assert(m_configuration);
        bool penetration = false;
        doAbsoluteRestingTest();
        //--- Mark all overlaps detected in this iteration by
        //--- the broad phase collision detection algorithm as
        //--- being alive and fresh.
        indirect_edge_iterator begin(edges.begin());
        indirect_edge_iterator end(edges.end());
        for( indirect_edge_iterator edge = begin;   edge!=end;  ++edge )
        {
          edge->m_ccg_time_stamp = m_time_stamp;
          edge->prunned() = true;

          if(edge->get_body_A()->has_joint_to(edge->get_body_B()))
            continue;
          if(edge->get_body_A()->is_fixed() && edge->get_body_B()->is_fixed())
            continue;
          if(edge->get_body_A()->is_scripted() && edge->get_body_B()->is_scripted())
            continue;
          if(edge->get_body_A()->is_scripted() && edge->get_body_B()->is_fixed())
            continue;
          if(edge->get_body_A()->is_fixed() && edge->get_body_B()->is_scripted())
            continue;
          if(!edge->get_body_A()->is_active() || !edge->get_body_B()->is_active())
            continue;
          if(edge->get_body_A()->m_ccg_absolute_resting &&  edge->get_body_B()->m_ccg_absolute_resting )
          {
            edge->m_relative_resting = true;
            continue;
          }
          vector3_type r_a, r_b;
          quaternion_type Q_a, Q_b;
          edge->get_body_A()->get_position( r_a );
          edge->get_body_A()->get_orientation( Q_a );
          edge->get_body_B()->get_position( r_b );
          edge->get_body_B()->get_orientation( Q_b );
          edge->m_ccg_xformAtoB.setModelUpdate(r_a, Q_a, r_b, Q_b );
          //--- Test whetever relative placement of objects have
          //--- changed since last iteration
          real_type epsilon = OpenTissue::math::working_precision<real_type>();
          if( edge->m_ccg_xformAtoB.is_equal(edge->m_ccg_xformAtoB_prev,epsilon))
          {
            //--- A and B have not moved relatively to each other, so
            //--- we should be able to exploit both the narrow phase
            //--- and contact determination results that was previously
            //--- computed.
            edge->m_relative_resting = true;

            /*
            KE 06-10-2004: This is currently not supported!!!
            //--- Test for ``old'' penetrations
            penetration |= (edge->m_ccg_state==edge_type::penetrating);
            continue;
            */
          }
          else
          {
            //--- A and B moved relatively to each other, so we
            //--- have to put them through the collision detection
            //--- pipeline once again...
            edge->m_relative_resting = false;
            edge->m_ccg_xformAtoB_prev.set(edge->m_ccg_xformAtoB);
            edge->m_ccg_xformBtoA.inverse(edge->m_ccg_xformAtoB);
          }
          edge->prunned() = false;
        }
        return penetration;
      };

      /**
      * Absolute Resting Test.
      * This method examines each body in the configuration and tries to determine
      * wheter it has moved since last invocation. If not then the body is said to
      * be in absolute rest.
      *
      * The term absolute rest, reflects how geometry is percieved it has nothing
      * to do with the dynamic state of a body.
      */
      void doAbsoluteRestingTest(void)
      {
        //real_type accuracy = math::working_precision<real_type>();
        real_type accuracy = 0.001;
        //--- Test Absolute Resting.
        for(typename configuration_type::body_iterator body = m_configuration->body_begin();body!=m_configuration->body_end();++body)
        {
          //--- We only want to test absolute rest on active bodies
          if(!body->is_active())
            continue;
          //--- Now we are ready to test if the body is in absolute rest
          vector3_type r;
          quaternion_type Q;
          body->get_position( r );
          body->get_orientation( Q );
          if(
            r.is_equal(body->m_ccg_r_prev,accuracy)
            &&
            Q.is_equal(body->m_ccg_Q_prev,accuracy)
            )
          {
            body->m_ccg_absolute_resting = true;
          }
          else
          {
            body->m_ccg_absolute_resting = false;
            body->m_ccg_r_prev = r;
            body->m_ccg_Q_prev.set(Q);
          }
        }
      };

      /**
      * Post Narrow Phase Analysis.
      * In this analysis all body pairs (i.e. overlaps or edges) that were processed by the narrow
      * phase collision detection algorithm are examined.
      *
      * For each body pair the contact state is determined and all separated body pairs
      * are filtred from the set of processed body pairs. This is because it does not
      * make sense to run contact determination on a separated body pair.
      *
      * @param edges
      */
      void post_narrow_phase_analysis(edge_ptr_container & edges  )
      {
        using std::min;

        assert(m_configuration);
        real_type envelope = m_configuration->get_collision_envelope();
        indirect_edge_iterator begin(edges.begin());
        indirect_edge_iterator end(edges.end());
        for(indirect_edge_iterator edge=begin;edge!=end;++edge)
        {
          if(edge->prunned())
            continue;
          //--- Test if we can used cached results instead of recomputing them
          //if(edge->m_relativeResting)
          //{
          //  edge->prunned() = true;
          //  continue;
          //}
          //
          //--- KE 02-02-2003: The above test is obviously true if objects have not moved
          //--- in WCS, but if they have moved in WCS, but keept their relative
          //--- placement unchanged, their contact coordinate system might be
          //--- rotated. Two solutions exist to this problem
          //---
          //---   A) Recompute contact determination whenever
          //---      absolute orientation is changed.
          //---   B) Keep contact points in a local frame (CCS).
          //---
          //--- We take the easy way out, and recompute stuff whenever both objects
          //--- are not in absolute rest! That is:
          //
          //if(  edge->A->m_absolute_resting &&  edge->B->m_absolute_resting  )
          //{
          //  edge->prunned() = true;
          //  continue;
          //}
          //
          //--- However these bodypairs were already pruned in the post_broad_phase_analysis
          //--- method, so the code is unecessary
          edge->m_ccg_state = edge_type::undefined;
          real_type minimum = OpenTissue::math::detail::highest<real_type>();
          for(typename edge_type::contact_iterator contact = edge->contact_begin();contact != edge->contact_end();++contact)
          {
            minimum = min(contact->m_distance,minimum);
          }
          if(minimum > envelope)
          {
            edge->m_ccg_state = edge_type::separating;
            edge->prunned() = true;
            continue;
          }
          if(minimum < -envelope)
            edge->m_ccg_state = edge_type::penetrating;
          else
            edge->m_ccg_state = edge_type::touching;
        }
      };

      /**
      * Post Contact Determination Analysis.
      * In this phase all the geometrical proximity information that have been determined
      * prior in the collision detection engine pipeline is analyzed inorder to build a
      * higher level construct called contact groups. That is independent groups of contacts
      * and bodies. Independent implies that they can be simulated independently of each other.
      *
      * @param edges            The edges that were detected by the broad phase collision detection algorithm.
      * @param reportedGroups   Upon return this group container contains all the contact groups.
      */
      void post_contact_determination_analysis( edge_ptr_container & edges, group_ptr_container & reportedGroups )
      {
        for(typename group_ptr_container::iterator group = reportedGroups.begin();group!=reportedGroups.end();++group)
          delete (*group);
        reportedGroups.clear();
        //--- Perform a modified connected components search
        for(typename configuration_type::body_iterator body = m_configuration->body_begin();body!=m_configuration->body_end();++body)
        {
          body->m_tag = 0;
        }
        //--- Mark all edges we want to treat as white, and those we want to ignore as black
        indirect_edge_iterator begin(edges.begin());
        indirect_edge_iterator end(edges.end());
        for( indirect_edge_iterator edge = begin; edge!=end; ++edge )
        {
          assert(edge->m_ccg_time_stamp==m_time_stamp);
          edge->m_ccg_color = edge_type::white;
          if( edge->m_ccg_state == edge_type::separating)
            edge->m_ccg_color = edge_type::black;
          if( !edge->get_body_A()->is_active())
            edge->m_ccg_color = edge_type::black;
          if( !edge->get_body_B()->is_active())
            edge->m_ccg_color = edge_type::black;
          if( edge->get_body_A()->is_fixed() && edge->get_body_B()->is_fixed())
            edge->m_ccg_color = edge_type::black;
          if(edge->get_body_A()->is_scripted() && edge->get_body_B()->is_scripted())
            edge->m_ccg_color = edge_type::black;
          if(edge->get_body_A()->is_scripted() && edge->get_body_B()->is_fixed())
            edge->m_ccg_color = edge_type::black;
          if(edge->get_body_A()->is_fixed() && edge->get_body_B()->is_scripted())
            edge->m_ccg_color = edge_type::black;
          if(edge->get_body_A()->has_joint_to(edge->get_body_B()))
            edge->m_ccg_color = edge_type::white;
        }
        int groupTag = 1;
        for( indirect_edge_iterator edge = begin; edge!=end; ++edge )
        {
          if(edge->m_ccg_color == edge_type::white)
          {
            group_type * group = new group_type();
            reportedGroups.push_back(group);
            traverseGroup(groupTag,&(*edge),group);
            groupTag++;
          }
        }
        group_type * isolated = new group_type();
        for(typename configuration_type::body_iterator body = m_configuration->body_begin();body!=m_configuration->body_end();++body)
        {
          if(body->m_tag==0 && body->is_active() && !body->is_fixed() && !body->is_sleepy() && !body->is_scripted())
            isolated->m_bodies.push_back(&(*body));
        }
        if(isolated->size_bodies()>0)
          reportedGroups.push_back(isolated);
        else
          delete isolated;
      };

    protected:

      /**
      * Traverse Contact group_type.
      *
      * @param groupTag     A unique index identifying the contact groups that is currently being traversed.
      * @param edge         The current body pair of the contact group that is traversed.
      * @param group        Upon return this argument holds the traversed contact group.
      */
      void traverseGroup(const size_type groupTag,edge_type * edge,group_type * group)
      {
        edge->m_ccg_color=edge_type::grey;
        if(edge->get_body_A()->m_tag!=groupTag)
        {
          group->m_bodies.push_back(edge->get_body_A());
          edge->get_body_A()->m_tag = groupTag;
        }
        if(edge->get_body_B()->m_tag!=groupTag)
        {
          group->m_bodies.push_back(edge->get_body_B());
          edge->get_body_B()->m_tag = groupTag;
        }

        for(typename edge_type::contact_iterator contact = edge->contact_begin();contact!=edge->contact_end();++contact)
        {
          group->m_contacts.push_back( &(*contact) );
        }

        if(edge->get_body_A()->has_joint_to(edge->get_body_B()))
        {
          assert(edge->size_contacts()==0 || !"");

          for(typename body_type::indirect_joint_iterator joint = edge->get_body_A()->joint_begin();joint!=edge->get_body_A()->joint_end();++joint)
          {
            if(joint->get_socket_A()->get_body()==edge->get_body_B() || joint->get_socket_B()->get_body()==edge->get_body_B())
            {
              group->m_constraints.push_back(&(*joint));
            }
          }
        }
        if(!edge->get_body_A()->is_fixed() && !edge->get_body_A()->is_scripted())
        {
          typename body_type::indirect_edge_iterator begin(edge->get_body_A()->edge_begin());
          typename body_type::indirect_edge_iterator end(edge->get_body_A()->edge_end());
          for(typename body_type::indirect_edge_iterator neighbor=begin;neighbor!=end;++neighbor)
          {
            if(neighbor->m_ccg_color == edge_type::white && neighbor->m_ccg_time_stamp==m_time_stamp)
            {
              traverseGroup(groupTag,&(*neighbor),group);
            }
          }
        }
        if(!edge->get_body_B()->is_fixed() && !edge->get_body_A()->is_scripted())
        {
          typename body_type::indirect_edge_iterator begin(edge->get_body_B()->edge_begin());
          typename body_type::indirect_edge_iterator end(edge->get_body_B()->edge_end());
          for(typename body_type::indirect_edge_iterator neighbor=begin;neighbor!=end;++neighbor)
          {
            if(neighbor->m_ccg_color == edge_type::white && neighbor->m_ccg_time_stamp==m_time_stamp)
            {
              traverseGroup(groupTag,&(*neighbor),group);
            }
          }
        }
        edge->m_ccg_color=edge_type::black;
      };
    public:

      void clear()
      {
        m_time_stamp    = 0;
        this->m_configuration = 0;
      }

      void add(body_type * body){}

      void remove(body_type * body){}

      void init(configuration_type & configuration)
      {
        clear();
        m_configuration = &configuration;
        m_time_stamp    = 0;
      }

    };/* End class CachingContactGraphAnalysis */

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_CACHING_CONTACT_GRAPH_ANALYSIS_H
#endif
