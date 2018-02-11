#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_GRAVITY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_GRAVITY_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_force_interface.h>

namespace OpenTissue
{
  namespace mbd
  {

    template<typename mbd_types>
    class Gravity 
      : public ForceInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::body_type                 body_type;
      typedef typename mbd_types::math_policy::vector3_type vector3_type;

    protected:

      vector3_type m_acceleration;    ///< The gravitatial acceleration.

    public:

      Gravity()
        : m_acceleration(0,-9.81,0)
      {}

      virtual ~Gravity(){}

    public:

      void compute(body_type const & body,vector3_type & force,vector3_type & torque)
      {
        if(body.is_fixed())
          force.clear();
        else
        {
          force = m_acceleration * body.get_mass();
        }
        torque.clear();
      }

      void set_acceleration(vector3_type const & acceleration) { this->m_acceleration = acceleration; }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_GRAVITY_H
#endif 
