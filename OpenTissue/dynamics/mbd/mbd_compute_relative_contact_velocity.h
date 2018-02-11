#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_RELATIVE_CONTACT_VELOCITY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_RELATIVE_CONTACT_VELOCITY_H
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
    * Compute relative Contact Velocity.
    *
    * @param v_a  The linear velocity of the center of mass of object A.
    * @param w_a  The angular velocity around the center of mass of object A.
    * @param r_a  The arm from center of mas of object A to the point of contact.
    * @param v_b  The linear velocity of the center of mass of object B.
    * @param w_b  The angular velocity around the center of mass of object B.
    * @param r_b  The arm from center of mas of object B to the point of contact.
    * @return     The relative contact velocity of the two bodies.
    */
    template<typename vector3_type>
    vector3_type compute_relative_contact_velocity(
      vector3_type const & v_a
      , vector3_type const & w_a
      , vector3_type const & r_a
      , vector3_type const & v_b
      , vector3_type const & w_b
      , vector3_type const & r_b
      )
    {
      vector3_type u_a = cross(w_a , r_a) + v_a;
      vector3_type u_b = cross(w_b , r_b) + v_b;
      return (u_b - u_a);
    }

  } //End of namespace mbd

} //End of namespace OpenTissue

#endif // OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_RELATIVE_CONTACT_VELOCITY_H
