#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_ANGULAR_JOINT_LIMIT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_ANGULAR_JOINT_LIMIT_H
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

    template< typename mbd_types  >
    class AngularJointLimit 
      : public SubConstraintInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy::real_type         real_type;
      typedef typename mbd_types::math_policy::size_type         size_type;
      typedef typename mbd_types::math_policy::value_traits      value_traits;
      typedef typename mbd_types::math_policy::vector3_type      vector3_type;
      typedef typename mbd_types::math_policy::vector_range      vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range  idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range      matrix_range;

    protected:

      real_type    m_theta_min;    ///< Minimum displacement along joint axe away from calibration position ( theta_min <= theta_calibration=0 <= theta_max).
      real_type    m_theta_max;    ///< Maximum displacement along joint axe away from calibration position ( theta_min <= theta_calibration=0 <= theta_max).
      real_type    m_theta_cur;    ///< The current displacement along the joint axe away from calibration position.
      real_type    m_theta_error;  ///< The current displacement error along the joint axe. That is if slider have moved beyond it limits.
      vector3_type m_s_wcs;        ///< The joint axe in WCS.
      bool         m_low_active;   ///< Boolean flag indicating whetever the minimum limit is active or not.
      size_type    m_rows;         ///< Number of jacobian rows.
      real_type    m_gamma;        ///< Constraint force mixing.
      real_type    m_solution;     ///< Solution

    public:

      /**
      * Set Minimum Limit.
      *
      * @real_type theta_min     The new minimum limit (must be in range [-pi..0] or -infinity if unbounded).
      */
      void set_min_limit(real_type const & theta_min)
      {
        assert(theta_min == OpenTissue::math::detail::lowest<real_type>() || (theta_min<=value_traits::zero()  && theta_min>=-value_traits::pi()) || !"AngularJointLimit::set_min_limit(): min out of bounds");
        assert(m_theta_max>=theta_min || !"AngularJointLimit::set_min_limit(): min out of bounds");
        m_theta_min = theta_min;
      }

      real_type get_min_limit() const { return m_theta_min; }

      /**
      * Set Maximum Limit.
      *
      * @real_type theta_max     The new maximum limit (must be in range [0..pi] or infinity if unbounded).
      */
      void set_max_limit(real_type const & theta_max)
      {
        assert(theta_max == OpenTissue::math::detail::highest<real_type>() || (theta_max>=value_traits::zero()  && theta_max<=value_traits::pi()) || !"AngularJointLimit::set_max_limit(): max out of bounds");
        assert(theta_max>=m_theta_min || !"AngularJointLimit::set_max_limit(): max out of bounds");
        m_theta_max = theta_max;
      }

      real_type get_max_limit() const  { return m_theta_max; }

    public:

      AngularJointLimit()
        : m_theta_min( OpenTissue::math::detail::lowest<real_type>() )
        , m_theta_max( OpenTissue::math::detail::highest<real_type>() )
        , m_rows(0)
        , m_gamma(value_traits::zero())
        , m_solution(value_traits::zero())
      {}

      virtual ~AngularJointLimit(){}

    public:

      void evaluate(vector3_type const & s_wcs,real_type const & angle)
      {
        m_rows         = 0;
        m_theta_cur    = angle;
        m_s_wcs        = s_wcs;
        m_theta_error  = value_traits::zero();
        m_low_active   = false;

        if(m_theta_min != OpenTissue::math::detail::lowest<real_type>() && m_theta_cur<m_theta_min)
        {
          m_theta_error = m_theta_min - m_theta_cur;
          m_low_active  = true;
          m_rows        = 1;
        }
        if(m_theta_max != OpenTissue::math::detail::highest<real_type>() && m_theta_cur>m_theta_max)
        {
          m_theta_error = m_theta_max - m_theta_cur;
          m_rows        = 1;
        }
      }

      size_type get_number_of_jacobian_rows() const {return m_rows;}

      void get_linear_jacobian_A(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = value_traits::zero();
        J(offset,1) = value_traits::zero();
        J(offset,2) = value_traits::zero();
      }

      void get_linear_jacobian_B(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = value_traits::zero();
        J(offset,1) = value_traits::zero();
        J(offset,2) = value_traits::zero();
      }

      void get_angular_jacobian_A(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = -m_s_wcs(0);  J(offset,1) = -m_s_wcs(1); J(offset,2) = -m_s_wcs(2);
      }

      void get_angular_jacobian_B(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = m_s_wcs(0);  J(offset,1) = m_s_wcs(1);  J(offset,2) = m_s_wcs(2);
      }

      void get_stabilization_term(vector_range & b_error,size_type const & offset)const
      {
        b_error(offset) = this->get_frames_per_second()*this->get_error_reduction_parameter()*m_theta_error;
      }

      void get_low_limits(vector_range & lo,size_type const & offset)const
      {
        if(m_low_active)
          lo(offset) = value_traits::zero();
        else
          lo(offset) = OpenTissue::math::detail::lowest<real_type>();
      }

      void get_high_limits(vector_range & hi,size_type const & offset)const
      {
        if(m_low_active)
          hi(offset) = OpenTissue::math::detail::highest<real_type>();
        else
          hi(offset) = value_traits::zero();
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
// OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_ANGULAR_JOINT_LIMIT_H
#endif
