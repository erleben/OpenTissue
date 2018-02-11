#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_EXHAUSTIVE_SEARCH_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_EXHAUSTIVE_SEARCH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <list>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * The Exhaustive Search Broad Phase Collision Detection Algorithm.
    *
    * The algorithm is intended to be used as a policy in the CollisionDetection class.
    */
    template<typename types>
    class ExhaustiveSearch
    {
    protected:

      //--- Convience Stuff for better readability
      typedef typename types::configuration_type configuration_type;
      typedef typename types::body_type body_type;
      typedef typename types::edge_type edge_type;
      typedef typename types::edge_ptr_container edge_ptr_container;
      typedef typename types::math_policy::real_type      real_type;
      typedef typename types::math_policy::vector3_type    vector3_type;
      typedef typename types::math_policy::matrix3x3_type  matrix3x3_type;

    protected:

      configuration_type * m_configuration;         ///< A pointer to the configuration that holds the configuration
      ///< that the broad phase collision detection algorithm is
      ///< intended to work on.
    public:

      class node_traits  
      {
      public:
        vector3_type m_es_pmin;  ///< Minimum corner of enclosing AABB
        vector3_type m_es_pmax;  ///< Maximum corner of enclosing AABB
      };
      class edge_traits  {   };
      class constraint_traits { };

    public:

      ExhaustiveSearch()
        : m_configuration(0) 
      {}

    public:

      void clear()
      {
        this->m_configuration = 0;
      }

      void init(configuration_type & configuration) {  m_configuration = &configuration;  };
      void add(body_type * /*body*/)  {  };
      void remove(body_type * /*body*/)  {    };

      /**
      * Run Exhaustive Search.
      *
      * @param edges   Upon return this argument holds all the reported overlaps.
      */
      void run(edge_ptr_container & edges)
      {
        edges.clear();
        assert(m_configuration);
        typename configuration_type::body_iterator body1,body2,tmp;
        typename configuration_type::body_iterator begin = m_configuration->body_begin();
        typename configuration_type::body_iterator end = m_configuration->body_end();
        real_type envelope = m_configuration->get_collision_envelope();
        //--- Update AABB's of all bodies
        for(body1 = begin; body1!=end; ++body1)
        {
          vector3_type r;
          matrix3x3_type R;
          body1->get_position(r);
          body1->get_orientation(R);
          body1->compute_collision_aabb(r,R,body1->m_es_pmin,body1->m_es_pmax,envelope);
        }
        //--- Test AABB's of each body pair against each other for collision
        for(body1 = begin; body1!=end; ++body1)
        {
          tmp = body1; ++tmp;
          for(body2 = tmp; body2!=end; ++body2)
          {
            if(!body1->has_joint_to(  &(*body2) ))
            {
              if(body1->m_es_pmin(0)> body2->m_es_pmax(0) || body2->m_es_pmin(0)>body1->m_es_pmax(0))
                continue;
              if(body1->m_es_pmin(1)> body2->m_es_pmax(1) || body2->m_es_pmin(1)>body1->m_es_pmax(1))
                continue;
              if(body1->m_es_pmin(2)> body2->m_es_pmax(2) || body2->m_es_pmin(2)>body1->m_es_pmax(2))
                continue;
            }
            //--- Colliding AABB's were detected, report edge as ovelapping
            edge_type * edge = m_configuration->get_edge(&(*body1),&(*body2));
            if(!edge)
              edge = m_configuration->add(&(*body1),&(*body2));
            edges.push_back( edge );
          }
        }
      };
    };/* End class ExhaustiveSearch */

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_EXHAUSTIVE_SEARCH_H
#endif
