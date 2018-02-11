#ifndef OPENTISSUE_DYNAMICS_MULIBODY_CD_MBD_GEOMETRY_DISPATCHER_H
#define OPENTISSUE_DYNAMICS_MULIBODY_CD_MBD_GEOMETRY_DISPATCHER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/utility/dispatchers/dispatchers_dynamic_table_dispatcher.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Geometry Dispatcher.
    * This class should be used as a narrow phase collision detection
    * module in a collision ddetection engine.
    * This class merely tries to figure out what kind of geometries a pair of bodies
    * have and then call the appropriate collision detection algorithm for
    * the specific pair of geometries.
    *
    * The idea is to setup a table of pointers to so-called collider functions. The
    * geometry types of the objects are then used to lookup the collider that should
    * be invoked. This way the pairing of geometry types takes constant time, regardless
    * of how many geometry types one have.
    */
    template<typename mbd_types>
    class GeometryDispatcher
    {
    public:

      class node_traits { };
      class edge_traits { };
      class constraint_traits {};

    protected:

      typedef typename mbd_types::configuration_type     configuration_type;
      typedef typename mbd_types::geometry_type          geometry_type;
      typedef typename mbd_types::collision_info_type    collision_info_type;
      typedef typename mbd_types::body_type              body_type;
      typedef typename mbd_types::edge_type              edge_type;

      typedef OpenTissue::utility::dispatchers::DynamicTableDispatcher<geometry_type, false, bool, collision_info_type> dispatcher_type;

    protected:

      dispatcher_type      m_dispatcher;               ///< The dispatcher
      configuration_type * m_configuration;            ///< A pointer to the configuration.

    public:

      GeometryDispatcher() 
        : m_configuration(0)
      {}

    public:

      /**
       * This method binds a collision handler to the geometry dispatcher.
       *
       * @param f    The collision handler function. The first two arguments must
       *             be ``geometry types'' the third argument must be a collision
       *             info and the return value must be bool.
       */
      template < class T1, class T2 >
      void bind( bool (*f)(T1&, T2&, collision_info_type&)  )
      {
        m_dispatcher.bind( f );
      }

      /**
      * Run Narrow Phase Collision Detection Algorithm
      *
      * @param edge            A pointer to an edge represeting the body pair that should be proceesed.
      *
      * @return                    If a penetration was detected the return value is true otherwise it is false.
      */
      bool run( edge_type * edge )
      {
        assert(m_configuration || !"GeometryDispatcher::run(): configuration was NULL");
        assert(edge            || !"GeometryDispatcher::run(): edge was NULL");

        collision_info_type info( edge->get_body_A(), edge->get_body_B(), m_configuration->get_collision_envelope(), edge->get_material(), edge->get_contacts() );

        geometry_type & geometry_A = *(edge->get_body_A()->get_geometry());
        geometry_type & geometry_B = *(edge->get_body_B()->get_geometry());

        return m_dispatcher(geometry_A, geometry_B, info);
      }

    public:

      void add(body_type * /*body*/){}
      void remove(body_type * /*body*/){}
      
      void clear()
      {
        //this->m_dispatcher.clear();  //2008-05-15 kenny: Hmm, should we do something about this?
        this->m_configuration = 0;
      }

      void init( configuration_type & configuration )
      {
        clear();
        m_configuration = &configuration;
      }

    };

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MULIBODY_CD_MBD_GEOMETRY_DISPATCHER_H
#endif
