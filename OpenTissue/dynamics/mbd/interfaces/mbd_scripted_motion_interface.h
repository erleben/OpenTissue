#ifndef	OPENTISSUE_DYNAMICS_MBD_SCRIPTED_MBD_SCRIPTED_MOTION_INTERFACE_H
#define	OPENTISSUE_DYNAMICS_MBD_SCRIPTED_MBD_SCRIPTED_MOTION_INTERFACE_H
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
    * Scripted Motion (callback) Class.
    * Scripted motion types are implemented by inheritence from this
    * adapter, the compute method must be overriden and specified
    * for the wanted motion type. 
    *
    * Notice that a scripted motion is a type that can be applied to multiple
    * bodies. Invocation of the run method evaluates the
    * motion position and orientation on the specified body at a given instant of time.
    */
    template<typename mbd_types>
    class ScriptedMotionInterface 
      : public mbd_types::identifier_type
    {
    public:

      typedef typename mbd_types::body_type                      body_type;
      typedef typename mbd_types::math_policy::real_type         real_type;
      typedef typename mbd_types::math_policy::vector3_type      vector3_type;
      typedef typename mbd_types::math_policy::quaternion_type   quaternion_type;

    public:

      /**
      * Compute State of Scripted body_type.
      * 
      * @param body    A pointer to the rigid body where the
      *                scripted motion should be evaluated for.
      * @param time    The time at which the scripted motion
      *                should be evaluated.
      * @param r       Upon return this vector should contain
      *                the position of the center of mass.
      * @param q       Upon return this argument should contain
      *                the orientation representated as a quaternion.
      * @param v       Upon return this vector contains the value of
      *                the linear velocity of the center of mass.
      * @param w       Upon return this vector contains the value
      *                of the angular velocity.
      */
      virtual void compute(body_type const & body,real_type const & time,vector3_type & r,quaternion_type & q, vector3_type & v,vector3_type & w)=0;

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_SCRIPTED_MBD_SCRIPTED_MOTION_INTERFACE_H
#endif 
