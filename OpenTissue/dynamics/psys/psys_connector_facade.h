#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_CONNECTOR_FACADE_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_CONNECTOR_FACADE_H
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

  namespace psys
  {


    template<typename types>
    class ConnectorFacade
    {
    public:

      typedef typename types::system_type              system_type;

    protected:

      system_type * m_owner; ///< The particle system the costraint belong to.

    public:

      ConnectorFacade()
        : m_owner(0)
      {}

    public:

      /**
      * Connect Object.
      * This method makes sure that the object belongs to the specified particle cluster.
      *
      * @param owner     A reference to the particle cluster that should own the object.
      */
      void connect(system_type const & owner) { m_owner = const_cast<system_type*>(&owner); }

      /**
      * Disconnect Object.
      * This method removes the object from the particle cluster it belongs to.
      */
      void disconnect()    {      m_owner = 0;    }

      /**
      * Get Owner Pointer.
      *
      * @return   A pointer to the particle cluster owner.
      */
      system_type       *  /* const & */ owner()       {  return m_owner; }
      system_type const *  /* const & */ owner() const {  return m_owner; }

    };

  } // namespace psys

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PSYS_CONNECTOR_FACADE_H
#endif
