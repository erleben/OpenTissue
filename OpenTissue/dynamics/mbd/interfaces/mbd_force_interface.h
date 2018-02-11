#ifndef	OPENTISSUE_DYNAMICS_MBD_MBD_FORCE_INTERFACE_H
#define	OPENTISSUE_DYNAMICS_MBD_MBD_FORCE_INTERFACE_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>


namespace OpenTissue
{
  namespace mbd
  {
    /**
    * force_type (callback) Class.
    * force_type types are implemented by inheritence from this
    * adapter, the compute method must be overriden and specified
    * for the wanted force type. 
    *
    * Notice that a force is a type that can be applied to multiple
    * bodies, such as gravity which can be shared by all bodies in
    * a configuration. invocation of the run method evaluates the
    * force types effect on the specified body.
    */
    template<typename mbd_types>
    class ForceInterface 
      : public mbd_types::identifier_type
    {
    public:

      typedef typename mbd_types::body_type         body_type;
      typedef typename mbd_types::math_policy::vector3_type vector3_type;

    public:
      /**
      * Compute External force_type and Torque.
      * Add the effect of a external force type to the force
      * and torque accumulators of the specified rigid body.
      * 
      * @param body    A pointer to the rigid body where the
      *                external force type should be applied.
      * @param force   Upon return this vector should contain
      *                the force on the center of mass.
      * @param torque  Upon return this vector should contain
      *                the torque wrt. the center of mass.
      */
      virtual void compute(body_type const & body,vector3_type & force,vector3_type & torque)=0;

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_FORCE_INTERFACE_H
#endif 
