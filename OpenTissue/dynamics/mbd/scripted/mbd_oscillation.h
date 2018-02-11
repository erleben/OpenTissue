#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_SCRIPTED_MBD_OSCILLATION_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_SCRIPTED_MBD_OSCILLATION_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_scripted_motion_interface.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Oscillation Motion (callback) Class.
    *
    * Example of usage:
    *
    *  types::Sphere sphere;
    *  types::body_type body;
    *  types::Oscillation oscillation;
    *  types::configuration_type configuration;
    *
    *  oscillation.set_frequency(value_traits::pi()/4.);
    *  body.set_scripted_motion(&oscillation);
    *  body.set_geometry(sphere);
    *  oscillation.set_origin(vector3_type(0,sphere.radius(),0));
    *  configuration.add(&body);
    *
    * Notice that you have to add both a geometry and a scripted motion to a scripted object.
    */
    template<typename mbd_types>
    class Oscillation 
      : public ScriptedMotionInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::body_type            body_type;
      typedef typename mbd_types::math_policy          math_policy;
      typedef typename math_policy::value_traits         value_traits;
      typedef typename math_policy::real_type            real_type;
      typedef typename math_policy::vector3_type         vector3_type;
      typedef typename math_policy::quaternion_type      quaternion_type;

    protected:

      vector3_type m_origin;      ///< The origin. Default value is (0,0,0).
      vector3_type m_dir;         ///< The oscillation direction. Default value is (1,0,0).
      real_type    m_amplitude;   ///< The amplitude of the oscillation. Default value is 10.
      real_type    m_frequency;   ///< The frequency of the oscillation. Default value is 2 PI.
      real_type    m_phase;       ///< The phase shift of the oscillation. Default value is PI.

    public:

      Oscillation()
        : m_origin(value_traits::zero(),value_traits::zero(),value_traits::zero())
        , m_dir(value_traits::one(),value_traits::zero(),value_traits::zero())
        , m_amplitude(10*value_traits::one())
        , m_frequency(2*value_traits::pi())
        , m_phase(value_traits::pi())
      {}

      virtual ~Oscillation(){}

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
      void compute(body_type const & body,real_type const & time,vector3_type & r,quaternion_type & q, vector3_type & v,vector3_type & w)
      {
        using std::cos;
        using std::sin;

        r = m_dir*m_amplitude*cos(m_frequency*time + m_phase) + m_origin;
        v = - m_dir*m_frequency*m_amplitude*sin(m_frequency*time + m_phase);
        q.identity();
        w.clear();
      }

    public:

      /**
      * Set Origin.
      *
      * @parma origin    The new origin.
      */
      void set_origin(vector3_type const & origin){ m_origin = origin; }

      /**
      * Get Origin.
      *
      * @param origin    Upon return this vector argument holds the value of the current origin.
      */
      void get_origin(vector3_type & origin) const { origin = m_origin; }

      /**
      * Set oscillation direction.
      *
      * @parma dir    The new oscillation direction.
      */
      void set_direction(vector3_type const & dir) { m_dir = unit(dir); }

      /**
      * Get Oscillation Direction.
      *
      * @param dir    Upon return this vector argument holds a unit vector giving the direction of the oscillation.
      */
      void get_direction(vector3_type & dir) const { dir = m_dir; }

      void set_frequency(real_type const & frequency)
      {
        assert(frequency>=value_traits::zero() || !"Oscillation::set_frequency(): frequency must be non-negative");
        m_frequency = frequency;
      }

      real_type get_frequency() const { return m_frequency; }

      void set_phase(real_type const & phase) { m_phase = phase;  }

      real_type get_phase() const { return m_phase; }

      void set_amplitude(real_type const & amplitude)
      {
        assert(amplitude>=value_traits::zero() || !"Oscillation::set_amplitude(): amplitude must be non-negative");
        m_amplitude = amplitude;
      }

      real_type get_amplitude() const { return m_amplitude; }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_SCRIPTED_MBD_OSCILLATION_H
#endif 
