#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MOTORS_MBD_ANGULAR_JOINT_MOTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MOTORS_MBD_ANGULAR_JOINT_MOTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_sub_constraint_interface.h>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types >
    class AngularJointMotor 
      : public SubConstraintInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy::index_type        size_type;
      typedef typename mbd_types::math_policy::value_traits      value_traits;
      typedef typename mbd_types::math_policy::real_type         real_type;
      typedef typename mbd_types::math_policy::vector3_type      vector3_type;
      typedef typename mbd_types::math_policy::vector_range      vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range  idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range      matrix_range;

    protected:

      real_type    m_f_max;           ///< Maximum allowed force.
      real_type    m_v_desired;       ///< The desired velocity to move along joint axe.
      vector3_type m_s_wcs;           ///< The joint axe in WCS.
      bool         m_eval_active;     ///< Boolean flag indicating whetever the motor is active or not.
      size_type    m_rows;            ///< Number of jacobian rows.
      real_type    m_gamma;           ///< Constraint force mixing.
      real_type    m_solution;        ///< Solution

    public:

      void      set_maximum_force(real_type const & f_max)           { m_f_max = f_max;         }
      real_type get_maximum_force()                            const { return m_f_max;          }
      void      set_desired_speed(real_type const & v_desired)       { m_v_desired = v_desired; }
      real_type get_desired_speed()                            const { return m_v_desired;      }

    public:

      AngularJointMotor()
        : m_f_max(value_traits::zero())
        , m_v_desired(value_traits::zero())
        , m_rows(0)
        , m_gamma(value_traits::zero())
        , m_solution(value_traits::zero())
      {}

      virtual ~AngularJointMotor(){}

    public:

      void evaluate(vector3_type const & s_wcs)
      {
        m_rows = 0;
        if(m_v_desired == OpenTissue::math::detail::lowest<real_type>() || m_v_desired == OpenTissue::math::detail::highest<real_type>() )
          return;
        if(m_f_max==0)
          return;
        m_rows = 1;
        m_s_wcs = s_wcs;
      }

      size_type get_number_of_jacobian_rows() const { return m_rows; }

      void get_linear_jacobian_A(matrix_range & J, size_type const & offset) const
      {
        J(offset,0) = value_traits::zero();
        J(offset,1) = value_traits::zero();
        J(offset,2) = value_traits::zero();
      }

      void get_linear_jacobian_B(matrix_range & J, size_type const & offset) const
      {
        J(offset,0) = value_traits::zero();
        J(offset,1) = value_traits::zero();
        J(offset,2) = value_traits::zero();
      }

      void get_angular_jacobian_A(matrix_range & J, size_type const & offset) const
      {
        J(offset,0) = -m_s_wcs(0);    J(offset,1) = -m_s_wcs(1);    J(offset,2) = -m_s_wcs(2);
      }

      void get_angular_jacobian_B(matrix_range & J, size_type const & offset) const
      {
        J(offset,0) = m_s_wcs(0);    J(offset,1) = m_s_wcs(1);    J(offset,2) = m_s_wcs(2);
      }

      void get_stabilization_term(vector_range & b_error, size_type const & offset) const 
      { 
        b_error(offset) = m_v_desired; 
      }

      void get_low_limits(vector_range & lo,size_type const & offset) const 
      { 
        lo(offset) = -m_f_max/this->get_frames_per_second(); 
      }

      void get_high_limits(vector_range & hi,size_type const & offset) const 
      { 
        hi(offset) = m_f_max/this->get_frames_per_second(); 
      }

      void get_dependency_indices(idx_vector_range & dep,size_type const & offset) const 
      {
        dep(offset) = OpenTissue::math::detail::highest<size_type>();  
      }

      void get_dependency_factors(vector_range & factors,size_type const & offset) const 
      { 
        factors(offset) = value_traits::zero(); 
      }

      void set_regularization(vector_range const & gamma,size_type const & offset)
      { 
        m_gamma = gamma(offset); 
      }

      void get_regularization(vector_range & gamma,size_type const & offset) const 
      {
        gamma(offset) = m_gamma;  
      }

      void set_solution(vector_range const & solution,size_type const & offset) 
      {  
        m_solution = solution(offset); 
      }

      void get_solution(vector_range & solution,size_type const & offset) const 
      {
        solution(offset) = m_solution; 
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MOTORS_MBD_ANGULAR_JOINT_MOTOR_H
#endif
