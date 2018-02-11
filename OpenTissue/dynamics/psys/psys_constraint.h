#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_CONSTRAINT_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_CONSTRAINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/psys_connector_facade.h>

namespace OpenTissue
{

  namespace psys
  {

    /**
    * Constraint Base Class.
    */
    template<typename types>
    class Constraint
      : public ConnectorFacade<  types >
    {
    public:

      typedef typename types::system_type    system_type;

    public:
      
      virtual ~Constraint(){};
      
    public:

      /**
      * Satisfy Constraint.
      * This method is invoked by caller.
      *
      * The method is responsible for relaxing the constraint on all
      * affected particles, that is it should adjust the positions of all
      * affected particles in a way such that they fulfill the constraint.
      */
      virtual void satisfy() 
      {
        assert(!"Constraint::satisfy(): Missing implementation");
      }
    };

  } // namespace psys

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PSYS_CONSTRAINT_H
#endif
