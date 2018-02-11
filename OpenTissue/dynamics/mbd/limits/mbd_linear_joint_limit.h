#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_LINEAR_JOINT_LIMIT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_LINEAR_JOINT_LIMIT_H
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
    class LinearJointLimit
      : public SubConstraintInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy::real_type          real_type;
      typedef typename mbd_types::math_policy::size_type          size_type;
      typedef typename mbd_types::math_policy::vector3_type       vector3_type;
      typedef typename mbd_types::math_policy::vector_range       vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range   idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range       matrix_range;
      typedef typename mbd_types::math_policy::value_traits       value_traits;

    protected:

      real_type     m_d_min;        ///< Minimum displacement along joint axe away from calibration position ( d_min <= d_calibration=0 <= d_max).
      real_type     m_d_max;        ///< Maximum displacement along joint axe away from calibration position ( d_min <= d_calibration=0 <= d_max).
      real_type     m_d_cur;        ///< The current displacement along the joint axe away from calibration position.
      real_type     m_d_error;      ///< The current displacement error along the joint axe. That is if slider have moved beyond it limits.
      vector3_type  m_s_wcs;        ///< The joint axe in WCS.
      vector3_type  m_c;            ///< body_type center displacement, ie. c = r_B - r_A
      vector3_type  m_half_cXs;     ///< 0.5*c X s_wcs
      bool          m_low_active;   ///< Boolean flag indicating whetever the minimum limit is active or not.
      size_type     m_rows;         ///< Number of jacobian rows.
      real_type     m_gamma;        ///< Constraint force mixing.
      real_type     m_solution;     ///< Solution

    public:

      /**
      * Set Minimum Limit.
      *
      * @real_type d_min     The new minimum limit (must be non-positive).
      */
      void set_min_limit(real_type const & d_min)
      {
        assert(d_min<=0 || !"LinearJointLimit::set_min_limit(): out of bounds");
        assert(d_min!=m_d_max || !"LinearJointLimit::set_min_limit(): out of bounds");
        m_d_min = d_min;
      }

      /**
      * Get Minimum Limit.
      *
      * @return    The current minimum limit.
      */
      real_type get_min_limit() const { return m_d_min; }

      /**
      * Set Maximum Limit.
      *
      * @real_type d_max     The new maximum limit (must be non-negative).
      */
      void set_max_limit(real_type const & d_max)
      {
        assert(0<=d_max || !"LinearJointLimit::set_max_limit(): out of bounds");
        assert(d_max!=m_d_min || !"LinearJointLimit::set_max_limit(): out of bounds");
        m_d_max = d_max;
      }

      /**
      * Get Maximum Limit.
      *
      * @return   The current maximum limit.
      */
      real_type get_max_limit() const { return m_d_max; }

    public:

      LinearJointLimit()
        : m_d_min( OpenTissue::math::detail::lowest<real_type>() )
        , m_d_max( OpenTissue::math::detail::highest<real_type>() )
        , m_rows(0)
        , m_gamma(0)
        , m_solution(0)
      {}

      virtual ~LinearJointLimit() {}

    public:

      /**
      * Evaluate limit constraint.
      *
      * @param offset   joint_type offset (i.e. linear displacement from initial pose).
      * @param s_wcs    joint_type axis in world coordinate system.
      * @param r_A      Center of mass position of body A in world coordinate system.
      * @param r_B      Center of mass position of body B in world coordinate system.
      */
      void evaluate(real_type const & offset,vector3_type const & s_wcs,vector3_type const & r_A,vector3_type const & r_B )
      {
        m_rows = 0;

        if(m_d_min == OpenTissue::math::detail::lowest<real_type>() && m_d_max == OpenTissue::math::detail::highest<real_type>() )
          return;

        m_d_cur      = offset;
        m_d_error    = 0;
        m_low_active = false;
        if(m_d_min != OpenTissue::math::detail::lowest<real_type>() && m_d_cur <= m_d_min)
        {
          //--- Minimum limit is active
          m_d_error    = m_d_min - m_d_cur;
          m_rows       = 1;
          m_low_active = true;
        }
        if(m_d_max != OpenTissue::math::detail::highest<real_type>() && m_d_cur >= m_d_max)
        {
          //--- Maximum limit is active
          m_d_error = m_d_max - m_d_cur;
          m_rows    = 1;
        }

        m_s_wcs = s_wcs;
        m_c = r_B - r_A;
        m_half_cXs = cross(m_c , m_s_wcs)/value_traits::two();
      }

      size_type get_number_of_jacobian_rows() const {return m_rows;}

      void get_linear_jacobian_A(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = -m_s_wcs(0);    J(offset,1) = -m_s_wcs(1);    J(offset,2) = -m_s_wcs(2);
      }

      void get_linear_jacobian_B(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = m_s_wcs(0);    J(offset,1) = m_s_wcs(1);    J(offset,2) = m_s_wcs(2);
      }

      void get_angular_jacobian_A(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = m_half_cXs(0);    J(offset,1) = m_half_cXs(1);    J(offset,2) = m_half_cXs(2);
      }

      void get_angular_jacobian_B(matrix_range & J,size_type const & offset)const
      {
        J(offset,0) = -m_half_cXs(0);    J(offset,1) = -m_half_cXs(1);    J(offset,2) = -m_half_cXs(2);
      }

      void get_stabilization_term(vector_range & b_error,size_type const & offset)const
      {
        b_error(offset) = this->get_frames_per_second()*this->get_error_reduction_parameter()*m_d_error;
      }

      void get_low_limits(vector_range  & lo,size_type const & offset)const
      {
        if(m_low_active)
          lo(offset) = 0;
        else
          lo(offset) = OpenTissue::math::detail::lowest<real_type>();
      }

      void get_high_limits(vector_range & hi,size_type const & offset)const
      {
        if(m_low_active)
          hi(offset) = OpenTissue::math::detail::highest<real_type>();
        else
          hi(offset) = 0;
      }

      void get_dependency_indices(idx_vector_range & dep,size_type const & offset) const 
      { 
        dep(offset) = OpenTissue::math::detail::highest<size_type>(); 
      }

      void get_dependency_factors(vector_range & factors,size_type const & offset) const 
      { 
        factors(offset) = 0; 
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
// OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_LINEAR_JOINT_LIMIT_H
#endif
