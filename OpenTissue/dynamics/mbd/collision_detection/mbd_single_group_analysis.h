#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SINGLE_GROUP_ANALYSIS_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SINGLE_GROUP_ANALYSIS_H
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
    *
    */
    template<typename types>
    class SingleGroupAnalysis
    {
    protected:

      typedef typename types::math_policy::index_type      size_type;
      typedef typename types::math_policy::real_type       real_type;
      typedef typename types::math_policy::vector3_type    vector3_type;
      typedef typename types::math_policy::quaternion_type quaternion_type;
      typedef typename types::math_policy::coordsys_type   coordsys_type;

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
          : m_sga_r_prev(0,0,0)
          , m_sga_Q_prev(0,1,0,0)
          , m_sga_absolute_resting(false)
        {}

      public:
        vector3_type    m_sga_r_prev;      ///< The previous (from last invocation of collision detection engine) position of the node in WCS.
        quaternion_type m_sga_Q_prev;      ///< The previous (from last invocation of collision detection engine) orientation of the node in WCS.
        bool m_sga_absolute_resting;       ///< Boolean value indicating whatever this
        ///< body have been moved since the last invocation
        ///< of the collision detection engine. This
        ///< flag can be used to determine whatever
        ///<  caching could be exploited.
      };

      class edge_traits
      {
      public:

        typedef enum{
          undefined,        ///< State of body pair is not yet defined.
          separating,       ///< body_type pair is separated.
          touching,         ///< body_type pair is touching (but not penetrating).
          penetrating       ///< body_type pair is penetrating.
        } SGA_StateType;        ///< Type for the state of the body pair.

      public:
        edge_traits()
          : m_sga_time_stamp(0) 
        {}
      public:
        coordsys_type m_sga_xformAtoB;         ///< Storage holder for relative transform between A and B.
        coordsys_type m_sga_xformBtoA;         ///< Storage holder for relative transform between B and A.
        coordsys_type m_sga_xformAtoB_prev;    ///< Storage holder for the relative transform between A and B in the previous iteration of the contact analysis.
        size_type  m_sga_time_stamp;        ///< Time-stamp indicating when the edge was detected by the broad phase collision detection algorithm.
        SGA_StateType m_sga_state;             ///< The current contact state
      };

      class constraint_traits {  };


    protected:

      configuration_type * m_configuration;         ///< A pointer to the configuration that holds the current configuration.
      group_type m_group;                           ///< A body group used to collected isolated bodies
      size_type m_time_stamp;                       ///< Time-stamp indicating which iteration the STC analysis i performed in.

    public:

      SingleGroupAnalysis()
        : m_configuration(0)
        , m_time_stamp(0)
      {}

    protected:

      /**
      * Absolute Resting Test.
      * This method examines each body in the configuration and tries to determine
      * wheter it has moved since last invocation. If not then the body is said to
      * be in absolute rest.
      *
      * The term absolute rest, reflects how geometry is percieved it has nothing
      * to do with the dynamic state of a body.
      */
      void do_absolute_resting_test()
      {
        //real_type accuracy = precision;
        real_type accuracy = 0.001;      //--- TODO (KE 11-05-2005): Accuracy should be scale dependent (fraction of geometry size?)
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
            r.is_equal(body->m_sga_r_prev,accuracy)
            &&
            Q.is_equal(body->m_sga_Q_prev,accuracy)
            )
          {
            body->m_sga_absolute_resting = true;
          }
          else
          {
            body->m_sga_absolute_resting = false;
            body->m_sga_r_prev = r;
            body->m_sga_Q_prev = Q;
          }
        }
      }

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

        do_absolute_resting_test();

        //--- Mark all overlaps detected in this iteration by
        //--- the broad phase collision detection algorithm as
        //--- being alive and fresh.
        indirect_edge_iterator begin(edges.begin());
        indirect_edge_iterator end(edges.end());
        for( indirect_edge_iterator edge = begin;   edge!=end;  ++edge )
        {
          edge->m_sga_time_stamp = m_time_stamp;
          edge->prunned() = true;

          if(edge->get_body_A()->has_joint_to(edge->get_body_B()))
            continue;
          if(edge->get_body_A()->is_fixed() && edge->get_body_B()->is_fixed())
            continue;
          if(edge->get_body_A()->is_scripted() && edge->get_body_B()->is_scripted())
            continue;
          if(edge->get_body_A()->is_fixed() && edge->get_body_B()->is_scripted())
            continue;
          if(edge->get_body_A()->is_scripted() && edge->get_body_B()->is_fixed())
            continue;
          if(!edge->get_body_A()->is_active() || !edge->get_body_B()->is_active())
            continue;
          if(edge->get_body_A()->m_sga_absolute_resting &&  edge->get_body_B()->m_sga_absolute_resting )
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
          edge->m_sga_xformAtoB = model_update(r_a, Q_a, r_b, Q_b );
          //--- Test whetever relative placement of objects have
          //--- changed since last iteration
          static real_type threshold = OpenTissue::math::working_precision<real_type>();
          if( edge->m_sga_xformAtoB.is_equal(edge->m_sga_xformAtoB_prev,threshold)) //--- TODO (KE 11-05-2005): Precision should be scale dependent (fraction of smallest geometry size?)
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
            edge->m_sga_xformAtoB_prev = edge->m_sga_xformAtoB;
            edge->m_sga_xformBtoA = inverse(edge->m_sga_xformAtoB);
          }
          edge->prunned() = false;
        }
        return penetration;
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
          //if(  edge->A->m_sga_absolute_resting &&  edge->B->m_sga_absolute_resting  )
          //{
          //  edge->prunned() = true;
          //  continue;
          //}
          //
          //--- However these bodypairs were already pruned in the post_broad_phase_analysis
          //--- method, so the code is unecessary
          edge->m_sga_state = edge_type::undefined;
          real_type minimum = OpenTissue::math::detail::highest<real_type>();
          for(typename edge_type::contact_iterator contact = edge->contact_begin();contact != edge->contact_end();++contact)
          {
            minimum = min(contact->m_distance,minimum);
          }
          if(minimum > envelope)
          {
            edge->m_sga_state = edge_type::separating;
            edge->prunned() = true;
            continue;
          }
          if(minimum < -envelope)
            edge->m_sga_state = edge_type::penetrating;
          else
            edge->m_sga_state = edge_type::touching;
        }
      };

      void post_contact_determination_analysis( edge_ptr_container & edges, group_ptr_container & groups)
      {
        groups.clear();

        m_group.clear();
        for(typename configuration_type::body_iterator body = m_configuration->body_begin();body!=m_configuration->body_end();++body)
        {
          if(body->is_active())
            m_group.m_bodies.push_back( &(*body));
        }
        indirect_edge_iterator begin(edges.begin());
        indirect_edge_iterator end(edges.end());
        for(indirect_edge_iterator edge = begin;edge!=end;++edge )
        {
          for(typename edge_type::contact_iterator contact = edge->contact_begin();contact!=edge->contact_end();++contact)
          {
            m_group.m_contacts.push_back( &(*contact) );
          }

          body_type * A = edge->get_body_A();
          body_type * B = edge->get_body_B();
          if(A->has_joint_to(B))
          {
            for(typename body_type::indirect_joint_iterator joint = A->joint_begin();joint!=A->joint_end();++joint)
            {
              if(joint->get_socket_A()->get_body()==B || joint->get_socket_B()->get_body()==B)
              {
                m_group.m_constraints.push_back( &(*joint) );
              }
            }
          }
        }
        groups.push_back(&m_group);
      };

    public:

      void clear()  
      {
        this->m_time_stamp = 0;
        this->m_configuration = 0;
        this->m_group.clear();
      }

      void add(body_type * /*body*/){};
      void remove(body_type * /*body*/){};
      void init(configuration_type & configuration)
      {
        clear();
        m_configuration = &configuration;
        m_time_stamp = 0;
      };

    };

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SINGLE_GROUP_ANALYSIS_H
#endif
