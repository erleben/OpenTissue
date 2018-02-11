#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_COLLISION_DETECTION_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_COLLISION_DETECTION_H
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
    * Collision Detection Engine.
    * This class implements the basic behavior of a traditional collision
    * detection engine, there are some subtelty to it, which is not seen
    * in most others collsion detection engines. The different thing is
    * that there is an explicit contact determination phase and a spatical-temporal
    * analysis phase (see DIKU technical report no. 04-06 for more details).
    */
    template<
      typename types,                                                ///< This is suppsed to be the TypeBinder.
      template<typename types> class BroadPhasePolicy,               ///< The broad phase collision detection that should be used.
      template<typename types> class NarrowPhasePolicy,              ///< The narrow phase collision detection that should be used.
      template<typename types> class AnalyzerPolicy                  ///< The Spatial Temporal Analysis that should be used.
    >
    class CollisionDetection
    {
    protected:

      typedef typename types::math_policy::index_type  size_type;
      typedef typename types::configuration_type       configuration_type;
      typedef typename types::body_type                body_type;
      typedef typename types::edge_type                edge_type;
      typedef typename types::group_ptr_container      group_ptr_container;
      typedef typename types::group_type               group_type;
      typedef typename types::edge_ptr_container       edge_ptr_container;
      typedef typename types::indirect_edge_iterator   indirect_edge_iterator;

      typedef BroadPhasePolicy<types>                  broad_phase_type;
      typedef NarrowPhasePolicy<types>                 narrow_phase_type;
      typedef AnalyzerPolicy<types>                    analyzer_type;

    public:

      class node_traits 
        : public broad_phase_type::node_traits
        , public narrow_phase_type::node_traits
        , public analyzer_type::node_traits
      {};

      class edge_traits 
        : public broad_phase_type::edge_traits
        , public narrow_phase_type::edge_traits
        , public analyzer_type::edge_traits
      {};

      class constraint_traits 
        : public broad_phase_type::constraint_traits
        , public narrow_phase_type::constraint_traits
        , public analyzer_type::constraint_traits
      {};

    protected:

      configuration_type *         m_configuration;            ///< A pointer to the configuration.
      broad_phase_type             m_broad_phase;              ///< Broad phase collision detection algorithm.
      narrow_phase_type            m_narrow_phase;             ///< Narrow phase collision detection algorithm.
      analyzer_type                m_analyzer;                 ///< Spatial Temporal Analyzer.
      bool                         m_short_circuit;            ///< Boolean flag indicating whether the collision detection engine should short-circuit on first penetration it finds.
      size_type                    m_time_stamp;               ///< Time-stamp of last invocation of the collision detection engine.

    public:

      CollisionDetection()
        : m_configuration(0)
        , m_short_circuit(false)
        , m_time_stamp(0)
      {}

    public:

      /**
      * Get Broad Phase Module.
      *
      * @return     A pointer to the broad phase collision detection module.
      */
      broad_phase_type       * get_broad_phase()       {  return &m_broad_phase; }
      broad_phase_type const * get_broad_phase() const {  return &m_broad_phase; }

      /**
      * Get Narrow Phase Module.
      *
      * @return     A pointer to the narrow phase collision detection module.
      */
      narrow_phase_type       * get_narrow_phase()       {  return &m_narrow_phase; }
      narrow_phase_type const * get_narrow_phase() const {  return &m_narrow_phase; }

      /**
      * Get Analyzer Module.
      *
      * @return     A pointer to the analyzer collision detection module.
      */
      analyzer_type       * get_analyzer()       {  return &m_analyzer; }
      analyzer_type const * get_analyzer() const {  return &m_analyzer; }


    public:

      /**
      * Run Collision Detection Query.
      *
      * @param groups    Upon return this container contains all the contact groups that have been detected.
      *
      * @return          If a penetration have been discovered then the return value is true otherwise it is false.
      */
      bool run(group_ptr_container & groups)
      {
        ++m_time_stamp;

        assert(m_configuration || !"CollisionDetection::run(): missing configuration");

        edge_ptr_container edges;
        bool penetration = false;
        m_broad_phase.run(edges);


        indirect_edge_iterator begin(edges.begin());
        indirect_edge_iterator end(edges.end());
        //--- Make sure to set up-to-date time stamp, see method BodyPair::is_up_to_date() for more information
        for(indirect_edge_iterator edge = begin;edge!=end;++edge)
        {
          edge->m_updated_time_stamp = m_time_stamp;
        }


        {
          // Add any missing edges that corresponds to joints between bodies!
          //
          // Hmmm, this does not seem to be a very efficient way of doing it!!!
          //
          typename configuration_type::joint_iterator joint = m_configuration->joint_begin();
          typename configuration_type::joint_iterator end = m_configuration->joint_end();
          for(;joint!=end;++joint)
          {
            body_type * A = joint->get_socket_A()->get_body();
            body_type * B = joint->get_socket_B()->get_body();
            edge_type * edge = m_configuration->get_edge(A,B);
            if(!edge)
              edge = m_configuration->add(A,B);
            if(edge->m_updated_time_stamp != m_time_stamp)
            {
              edge->m_updated_time_stamp = m_time_stamp;
              edges.push_back( &(*edge) );
            }
          }
        }

        penetration |= m_analyzer.post_broad_phase_analysis(edges);

        if(m_short_circuit && penetration)
          return penetration;

        for(indirect_edge_iterator edge = begin;edge!=end;++edge)
        {
          if(edge->prunned())
            continue;

          penetration |= m_narrow_phase.run(&(*edge));

          if(m_short_circuit && penetration)
            return penetration;
        }
        m_analyzer.post_narrow_phase_analysis(edges);

        m_analyzer.post_contact_determination_analysis(edges,groups);
        return penetration;
      }

      /**
      * Initialization Routine.
      *
      * @param configuration
      */
      void init(configuration_type & configuration)
      {
        m_time_stamp = 0;
        m_configuration = &configuration;
        m_broad_phase.init(configuration);
        m_narrow_phase.init(configuration);
        m_analyzer.init(configuration);
      }

      /**
      * Clear All.
      */
      void clear()
      {
        m_time_stamp = 0;
        m_broad_phase.clear();
        m_narrow_phase.clear();
        m_analyzer.clear();
      }

      /**
      * Notify Add body_type.
      * Used by the configuration to tell the collision detection engine that the configuration has changed.
      *
      * @param body
      * @return               A boolean value indicating whether the notification was succesfull or not.
      */
      bool add(body_type * body)
      {
        assert(body            || !"CollisionDetection::add(): body was null");
        assert(m_configuration || !"CollisionDetection::add(): missing configuration");
        m_broad_phase.add(body);
        m_narrow_phase.add(body);
        m_analyzer.add(body);
        return true;
      }

      /**
      * Notify Remove body_type.
      * Used by the configuration to tell the collision detection engine that the configuration has changed.
      *
      * @param body
      * @return               A boolean value indicating whether the notification was succesfull or not.
      */
      bool remove(body_type * body)
      {
        assert(body            || !"CollisionDetection::remove(): body was null");
        assert(m_configuration || !"CollisionDetection::remove(): missing configuration");
        m_broad_phase.remove(body);
        m_narrow_phase.remove(body);
        m_analyzer.remove(body);
        return true;
      }


      /**
      * Set Short Circuiting.
      *
      * @param value     The new short circuiting value, if on collision detection
      *                  engine will terminate the query first time a penetration
      *                  is encountered.
      */
      void set_short_circuiting(bool const & value)
      {
        m_short_circuit = value;
      }

      /**
      * Retrive Short Circuiting State.
      *
      * @return   A boolean value indicating whether short circuiting is on.
      */
      bool is_short_circuiting() const {return m_short_circuit;}

      /**
      * Get Time-Stamp.
      *
      * @return     The value of the time-stamp of the last invocation
      *             of the collision detection engine.
      */
      size_type get_time_stamp() const {return m_time_stamp;}

    };

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_COLLISION_DETECTION_H
#endif
