#ifndef OPENTISSUE_DYNAMICS_MBD_INTERFACES_MBD_CORE_CONSTRAINT_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_INTERFACES_MBD_CORE_CONSTRAINT_INTERFACE_H
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
  namespace mbd
  {
    template< typename mbd_types  >
    class CoreConstraintInterface 
      : public mbd_types::constraint_traits
    {
    public:

      typedef typename mbd_types::math_policy::index_type         size_type;
      typedef typename mbd_types::math_policy::real_type          real_type;
      typedef typename mbd_types::math_policy::value_traits       value_traits;

    protected:

      real_type m_erp;            ///< Error reduction parameter (must be in the range 0..1).
      real_type m_fps;            ///< Frames per second (used to compute the amount of error correction that should be done within the next timestep).
      bool      m_active;         ///< Boolean flag indicating if constraint is set to active or not (this has nothing to to do with the evalute method).
      bool      m_use_erp; 

    public:

      CoreConstraintInterface()
        : m_erp(value_traits::zero())
        , m_fps(value_traits::zero())
        , m_active(true) 
        , m_use_erp(true) 
      {}

      virtual ~CoreConstraintInterface(){};
      
    public:

      /**
      * Set Frames per Second Parameter.
      *
      * @param fps     The new value of the frames per second parameter.
      */
      void set_frames_per_second(real_type const & fps)
      {
        assert(fps>=0 || !"CoreConstraintInterface::set_frames_per_second(): fps must be non-negative");
        m_fps = fps;
      }

      /**
      * Get Frames Per Second.
      *
      * @return   The value of frames per second parameter.
      */
      real_type const & get_frames_per_second( )const { return m_fps; }

      /**
      * Set Error Reduction Parameter.
      *
      * @param erp     The new value of the error reduction parameter must be in the interval 0..1
      */
      void set_error_reduction_parameter(real_type const & erp)
      {
        assert( ( erp>=value_traits::zero() && erp<=value_traits::one() ) || !"CoreConstraintInterface::set_frames_per_second(): erp must be in [0..1]");
        m_erp = erp;
      }

      /**
      * Get Error Reduction Parameter.
      *
      * @return   The value of the error reduction parameter.
      */
      virtual real_type get_error_reduction_parameter() const 
      {
        if(this->use_erp())
          return m_erp; 
        return value_traits::one();
      }

      bool       & use_erp()       { return m_use_erp; }
      bool const & use_erp() const { return m_use_erp; }

      /**
      * Get Active State.
      * This method depends on the isEvaluatedActive() method and get_number_of_jacobian_rows(). If
      * during evaluation the constraint is evaluated to be non-active
      * then this takes precedence over the user setting.
      *
      * @return    The current active state.
      */
      bool is_active() const
      {
        if(!get_number_of_jacobian_rows())
          return false;
        return m_active;
      }

      /**
      * Set Active.
      * This method can be used by an end user to force a
      * constraint to be inactive without having to remove
      * the constraint from the configuration.
      *
      * This could for instance be useful to model
      * a joint breaking.
      *
      * @param active   The new active state.
      */
      void set_active(bool active) { m_active = active; }

    public:

      /**
      * Return Number of Jacobian Rows.
      * The number of jacobian rows corresponds to the number of
      * jacobian variables.
      *
      * @return Jacobian rows.
      */
      virtual size_type get_number_of_jacobian_rows() const = 0;

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_INTERFACES_MBD_CORE_CONSTRAINT_INTERFACE_H
#endif
