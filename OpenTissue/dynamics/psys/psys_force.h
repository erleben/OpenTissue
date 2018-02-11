#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_FORCE_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_FORCE_H
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
    class Force 
      : public types::connector_type
    {
    public:

      Force() {}

      virtual ~Force(){}

    public:

      /**
      * Apply Force.
      * This method is invoked, when the force should be applied to any affected particles.
      */
      virtual void apply()=0;

    };

  } // namespace psys

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PSYS_FORCE_H
#endif
