#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_DAMPING_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_DAMPING_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_force_interface.h>

#include <boost/cast.hpp>  // needed for boost::numeric_cast

namespace OpenTissue
{
  namespace mbd
  {

    template<typename mbd_types>
    class Damping 
      : public ForceInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::body_type                 body_type;
      typedef typename mbd_types::math_policy::real_type    real_type;
      typedef typename mbd_types::math_policy::vector3_type vector3_type;

    protected:

      real_type m_linear_viscosity;   ///< Linear Viscosity Coefficient
      real_type m_angular_viscosity;  ///< Angular Viscosity Coefficient

    public:

      Damping()
        : m_linear_viscosity(boost::numeric_cast<real_type>( .001) )
        , m_angular_viscosity(boost::numeric_cast<real_type>( .001 ) )
      {}

      virtual ~Damping(){}

    public:

      /**
      * Compute Damping force_type
      *
      * @param body   The body on which the damping froce should be applied.
      * @param force  Upon return this argument holds the linear damping force that should be applied to the body.
      * @param torque Upon return this argument holds the damping torque that should be applied to the body.
      */
      void compute(body_type const & body,vector3_type & force,vector3_type & torque)
      {
        if(body.is_fixed())
        {
          force.clear();
          torque.clear();
        }
        else
        {
          vector3_type V,W;
          body.get_velocity(V);
          body.get_spin(W);
          force = -V*m_linear_viscosity;
          torque = -W*m_angular_viscosity;
        }
      }

      /**
      * Set Linear Viscosity.
      *
      * @param viscosity   The new viscosity value, must be a non-negative value.
      */
      void set_linear_viscosity(real_type const & viscosity)
      {
        assert(viscosity>=0 || !"Damping::set_linear_viscosity(): value must be non-negative");
        m_linear_viscosity = viscosity;
      }

      /**
      * Set Angular Viscosity.
      *
      * @param viscosity   The new viscosity value, must be a non-negative value.
      */
      void set_angular_viscosity(real_type const & viscosity)
      {
        assert(viscosity>=0 || !"Damping::set_angular_viscosity(): value must be non-negative");
        m_angular_viscosity = viscosity;
      }

      /**
      * Retrieve Linear Viscosity.
      *
      * @return The current value of the linear viscosity.
      */
      real_type const & get_linear_viscosity() const { return m_linear_viscosity; }

      /**
      * Retrieve Angular Viscosity.
      *
      * @return The current value of the angular viscosity.
      */
      real_type const & get_angular_viscosity() const { return m_angular_viscosity; }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_FORCES_MBD_DAMPING_H
#endif 
