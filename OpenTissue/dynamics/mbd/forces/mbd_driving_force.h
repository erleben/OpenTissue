#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_DRIVING_FORCE_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_DRIVING_FORCE_H
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
    /**
    * Driving force_type (callback) Class.
    *
    * This class implemens and oscillating force, i.e. a force which
    * magnitude is modulated by a cosine signal.
    *
    * The initial un-modulated force is represented as F = m G, where
    * m is the body mass and G is an initial ``gravitational'' acceleration.
    * The driving force is computed as:
    *  
    *  F(t) = m cos(\omega t + \phi) G
    *
    * This class was contributed by Ricardo Ortiz-Rosado (UIOWA) and
    * tweaked by Kenny Erleben.
    */
    template<typename mbd_types>
    class DrivingForce 
      : public ForceInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::body_type                 body_type;
      typedef typename mbd_types::math_policy::vector3_type vector3_type;
      typedef typename mbd_types::math_policy::real_type    real_type;
      typedef typename mbd_types::math_policy::value_traits value_traits;

    protected:

      vector3_type   m_G;            ///< The Initial Acceleration. Initially set to gravity in positive y-axis.
      real_type      m_frequency;    ///< The frequency of the driving cycle in (radians per second).
      real_type      m_phase;        ///< The phase transition of the driving force (initial displacement).
      real_type      m_time;         ///< The global simulation time, i.e. the time at which the driving force should be evaluated.

    public:

      DrivingForce()
        : m_G(value_traits::zero(),9.81,value_traits::zero())
        , m_frequency(value_traits::two()*value_traits::pi())
        , m_phase(value_traits::zero())
        , m_time(value_traits::zero())
      {}

      virtual ~DrivingForce(){}

    public:

      void compute(body_type const & body,vector3_type & force,vector3_type & torque)
      {
        using std::cos;

        torque.clear();
        if(body.is_fixed())
        {
          force.clear();
          return;
        }
        force = body.get_mass() * m_G * cos(m_frequency*m_time + m_phase);
      }

      /**
      * Set Global Simulation Time.
      * This method should be invoked whenever the system time is advanced.
      *
      * @param time    The global system simulation time.
      */
      void set_global_simulation_time(real_type const & time){ m_time = time; }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_DRIVING_FORCE_H
#endif 
