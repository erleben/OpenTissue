#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_UNIVERSAL_JOINT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_UNIVERSAL_JOINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_joint_interface.h>
#include <OpenTissue/dynamics/mbd/limits/mbd_angular_joint_limit.h>
#include <OpenTissue/dynamics/mbd/motors/mbd_angular_joint_motor.h>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace mbd
  {

    template < typename mbd_types >
    class UniversalJoint 
      : public JointInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy                      math_policy;
      typedef typename mbd_types::math_policy::index_type         size_type;
      typedef typename mbd_types::math_policy::real_type          real_type;
      typedef typename mbd_types::math_policy::vector3_type       vector3_type;
      typedef typename mbd_types::math_policy::matrix3x3_type     matrix3x3_type;
      typedef typename mbd_types::math_policy::quaternion_type    quaternion_type;
      typedef typename mbd_types::math_policy::vector_range       vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range   idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range       matrix_range;
      typedef typename mbd_types::math_policy::vector_type        vector_type;
      typedef typename mbd_types::math_policy::value_traits       value_traits;

      typedef AngularJointLimit<mbd_types>    angular_limit_type;
      typedef AngularJointMotor<mbd_types>    angular_motor_type;

    protected:

      vector3_type    m_s_A_wcs;         ///< The joint axe wrt. to body A in WCS.
      vector3_type    m_s_B_wcs;         ///< The joint axe wrt. to body B in WCS.
      vector3_type    m_u;               ///< Error correction rotation axe. Ie. if joint is misaligned this vector is used to determine an rotation axe which can be used to align the joint along.
      matrix3x3_type  m_star_anc_A;      ///< The cross product operator of the anchor point wrt. to body A rotated (but not translated) into WCS.
      matrix3x3_type  m_star_anc_B;      ///< The cross product operator of the anchor point wrt. to body B rotated (but not translated) into WCS.
      vector3_type    m_anc_A_wcs;       ///< The anchor point wrt. to body A in WCS.
      vector3_type    m_anc_B_wcs;       ///< The anchor point wrt. to body B in WCS.
      quaternion_type m_Q_initial;       ///< The initial relative orientation between the two bodies (wrt. WCS) i.e. the rotation that aligns body A with body B.
      quaternion_type m_Q_initial_conj;  ///< The conjugate of Q_initial.
      size_type       m_rows;            ///< Number of jacobian rows.
      vector_type     m_gamma;           ///< Local vector of constraint force mixing terms.
      vector_type     m_solution;        ///< Local solution vector, ie. vector of lagrange multipliers.

      angular_limit_type * m_limit1;
      angular_limit_type * m_limit2;
      angular_motor_type * m_motor1;
      angular_motor_type * m_motor2;

      size_type m_motor_offset1;
      size_type m_motor_offset2;
      size_type m_limit_offset1;
      size_type m_limit_offset2;
    
    public:

      /**
      * Get Rotation Angle around Axis 1.
      * This is inspired by Hinge::get_angle method, see this methods for details.
      */
      real_type get_angle1() const
      {
        assert( this->m_socketA || !"UniversalJoint::get_angle1(): socket A was null");
        assert( this->m_socketB || !"UniversalJoint::get_angle1(): socket B was null");

        quaternion_type Q_A, Q_B;
        
        this->m_socketA->get_body() ->get_orientation( Q_A );
        this->m_socketB->get_body() ->get_orientation( Q_B );
        
        quaternion_type Q_rel = ( conj( Q_B ) % Q_A ) % m_Q_initial_conj;
        return get_angle( Q_rel, conj( Q_B ).rotate( get_joint_axis1_world() ) );
      }

      /**
      * Get Angle1 Rate.
      * This method computes the (signed) speed by which the joint
      * angle changes its radial position around the first joint axe.
      *
      * @return   The angle rate.
      */
      real_type get_angle1_rate() const
      {
        assert( this->m_socketA || !"UniversalJoint::get_angle1_rate(): socket A was null");
        assert( this->m_socketB || !"UniversalJoint::get_angle1_rate(): socket B was null");

        this->m_socketA->get_body() ->get_spin( this->w_A );
        this->m_socketB->get_body() ->get_spin( this->w_B );
        
        return get_joint_axis1_world() * ( this->w_A - this->w_B );
      }

      /**
      * Get Rotation Angle around Axis 2.
      * This is inspired by Hinge::get_angle method, see this methods for details.
      */
      real_type get_angle2() const
      {
        assert( this->m_socketA || !"UniversalJoint::get_angle2(): socket A was null");
        assert( this->m_socketB || !"UniversalJoint::get_angle2(): socket B was null");

        quaternion_type Q_A, Q_B;

        this->m_socketA->get_body() ->get_orientation( Q_A );
        this->m_socketB->get_body() ->get_orientation( Q_B );

        quaternion_type Q_rel = ( conj( Q_B ) % Q_A ) % m_Q_initial_conj;

        return get_angle( Q_rel, conj( Q_B ).rotate( get_joint_axis2_world() ) );
      }

      /**
      * Get Angle2 Rate.
      * This method computes the (signed) speed by which the joint
      * angle changes its radial position around the second joint axe.
      *
      * @return   The angle rate.
      */
      real_type get_angle2_rate() const
      {
        assert( this->m_socketA || !"UniversalJoint::get_angle2_rate(): socket A was null");
        assert( this->m_socketB || !"UniversalJoint::get_angle2_rate(): socket B was null");

        this->m_socketA->get_body() ->get_spin( this->w_A );
        this->m_socketB->get_body() ->get_spin( this->w_B );
        return get_joint_axis2_world() * ( this->w_A - this->w_B );
      }

      vector3_type get_joint_axis1_world() const
      {
        return this->m_socketA->get_joint_axis_world();
      }

      vector3_type get_joint_axis2_world() const
      {
        return this->m_socketB->get_joint_axis_world();
      }

      void set_limit1( angular_limit_type const & limit )
      {
        m_limit1 = const_cast<angular_limit_type*>( &limit );
      }

      void set_limit2( angular_limit_type const & limit )
      {
        m_limit2 = const_cast<angular_limit_type*>( &limit );
      }

      void set_motor1( angular_motor_type const & motor )
      {
        m_motor1 = const_cast<angular_motor_type*>( &motor );
      }

      void set_motor2( angular_motor_type const & motor )
      {
        m_motor2 = const_cast<angular_motor_type*>( &motor );
      }

    public:

      UniversalJoint()
        : m_rows( 0 )
        , m_limit1( 0 )
        , m_limit2( 0 )
        , m_motor1( 0 )
        , m_motor2( 0 )
      {}

      virtual ~UniversalJoint(){}

    public:

      void evaluate()
      {
        m_rows = 0;

        if ( !( this->m_socketA && this->m_socketB ) )
          return ;

        m_rows = 4;

        m_s_A_wcs = this->m_socketA->get_joint_axis_world();
        m_s_B_wcs = this->m_socketB->get_joint_axis_world();

        quaternion_type Q;
        this->m_socketA->get_body() ->get_orientation( Q );
        m_star_anc_A = star( Q.rotate( this->m_socketA->get_anchor_local() ) );
        this->m_socketB->get_body() ->get_orientation( Q );
        m_star_anc_B = star( Q.rotate( this->m_socketB->get_anchor_local() ) );

        m_anc_A_wcs = this->m_socketA->get_anchor_world();
        m_anc_B_wcs = this->m_socketB->get_anchor_world();

        m_u = cross(m_s_A_wcs , m_s_B_wcs);

        m_limit_offset1 = 0;
        m_motor_offset1 = 0;
        if ( m_limit1 )
        {
          m_limit1->evaluate( get_joint_axis1_world(), get_angle1() );
          if ( m_limit1->is_active() )
          {
            m_limit_offset1 = m_rows;
            ++m_rows;
          }
        }
        if ( m_motor1 )
        {
          m_motor1->evaluate( get_joint_axis1_world() );
          if ( m_motor1->is_active() )
          {
            m_motor_offset1 = m_rows;
            ++m_rows;
          }
        }
        m_limit_offset2 = 0;
        m_motor_offset2 = 0;
        if ( m_limit2 )
        {
          m_limit2->evaluate( get_joint_axis2_world(), get_angle2() );
          if ( m_limit2->is_active() )
          {
            m_limit_offset2 = m_rows;
            ++m_rows;
          }
        }
        if ( m_motor2 )
        {
          m_motor2->evaluate( get_joint_axis2_world() );
          if ( m_motor2->is_active() )
          {
            m_motor_offset2 = m_rows;
            ++m_rows;
          }
        }
      }

      size_type get_number_of_jacobian_rows() const
      {
        return m_rows;
      }

      void get_linear_jacobian_A( matrix_range & J ) const
      {
        assert( J.size1() == m_rows || !"UniversalJoint::get_linear_jacobian_A(): incorrect dimensions");
        assert( J.size2() == 3 || !"UniversalJoint::get_linear_jacobian_A(): incorrect dimensions");
        J( 0, 0 ) = value_traits::one();
        J( 0, 1 ) = value_traits::zero();
        J( 0, 2 ) = value_traits::zero();
        J( 1, 0 ) = value_traits::zero();
        J( 1, 1 ) = value_traits::one();
        J( 1, 2 ) = value_traits::zero();
        J( 2, 0 ) = value_traits::zero();
        J( 2, 1 ) = value_traits::zero();
        J( 2, 2 ) = value_traits::one();
        J( 3, 0 ) = value_traits::zero();
        J( 3, 1 ) = value_traits::zero();
        J( 3, 2 ) = value_traits::zero();

        if ( m_limit_offset1 )
          m_limit1->get_linear_jacobian_A( J, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_linear_jacobian_A( J, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_linear_jacobian_A( J, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_linear_jacobian_A( J, m_motor_offset2 );
      }

      void get_linear_jacobian_B( matrix_range & J ) const
      {
        assert( J.size1() == m_rows || !"UniversalJoint::get_linear_jacobian_B(): incorrect dimensions");
        assert( J.size2() == 3 || !"UniversalJoint::get_linear_jacobian_B(): incorrect dimensions");
        J( 0, 0 ) = -value_traits::one();
        J( 0, 1 ) = value_traits::zero();
        J( 0, 2 ) = value_traits::zero();
        J( 1, 0 ) = value_traits::zero();
        J( 1, 1 ) = -value_traits::one();
        J( 1, 2 ) = value_traits::zero();
        J( 2, 0 ) = value_traits::zero();
        J( 2, 1 ) = value_traits::zero();
        J( 2, 2 ) = -value_traits::one();
        J( 3, 0 ) = value_traits::zero();
        J( 3, 1 ) = value_traits::zero();
        J( 3, 2 ) = value_traits::zero();

        if ( m_limit_offset1 )
          m_limit1->get_linear_jacobian_B( J, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_linear_jacobian_B( J, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_linear_jacobian_B( J, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_linear_jacobian_B( J, m_motor_offset2 );
      }

      void get_angular_jacobian_A( matrix_range & J ) const
      {
        assert( J.size1() == m_rows || !"UniversalJoint::get_angular_jacobian_A(): incorrect dimensions");
        assert( J.size2() == 3 || !"UniversalJoint::get_angular_jacobian_A(): incorrect dimensions");
        J( 0, 0 ) = -m_star_anc_A( 0, 0 );
        J( 0, 1 ) = -m_star_anc_A( 0, 1 );
        J( 0, 2 ) = -m_star_anc_A( 0, 2 );
        J( 1, 0 ) = -m_star_anc_A( 1, 0 );
        J( 1, 1 ) = -m_star_anc_A( 1, 1 );
        J( 1, 2 ) = -m_star_anc_A( 1, 2 );
        J( 2, 0 ) = -m_star_anc_A( 2, 0 );
        J( 2, 1 ) = -m_star_anc_A( 2, 1 );
        J( 2, 2 ) = -m_star_anc_A( 2, 2 );
        J( 3, 0 ) = m_u( 0 );
        J( 3, 1 ) = m_u( 1 );
        J( 3, 2 ) = m_u( 2 );
        if ( m_limit_offset1 )
          m_limit1->get_angular_jacobian_A( J, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_angular_jacobian_A( J, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_angular_jacobian_A( J, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_angular_jacobian_A( J, m_motor_offset2 );
      }

      void get_angular_jacobian_B( matrix_range & J ) const
      {
        assert( J.size1() == m_rows || !"UniversalJoint::get_angular_jacobian_B(): incorrect dimensions");
        assert( J.size2() == 3 || !"UniversalJoint::get_angular_jacobian_B(): incorrect dimensions");
        J( 0, 0 ) = m_star_anc_B( 0, 0 );
        J( 0, 1 ) = m_star_anc_B( 0, 1 );
        J( 0, 2 ) = m_star_anc_B( 0, 2 );
        J( 1, 0 ) = m_star_anc_B( 1, 0 );
        J( 1, 1 ) = m_star_anc_B( 1, 1 );
        J( 1, 2 ) = m_star_anc_B( 1, 2 );
        J( 2, 0 ) = m_star_anc_B( 2, 0 );
        J( 2, 1 ) = m_star_anc_B( 2, 1 );
        J( 2, 2 ) = m_star_anc_B( 2, 2 );
        J( 3, 0 ) = -m_u( 0 );
        J( 3, 1 ) = -m_u( 1 );
        J( 3, 2 ) = -m_u( 2 );
        if ( m_limit_offset1 )
          m_limit1->get_angular_jacobian_B( J, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_angular_jacobian_B( J, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_angular_jacobian_B( J, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_angular_jacobian_B( J, m_motor_offset2 );
      }

      void get_stabilization_term( vector_range & b_error ) const
      {
        assert( b_error.size() == m_rows || !"UniversalJoint::get_stabilization_term(): incorrect dimensions");

        real_type k_cor = this->get_frames_per_second()*this->get_error_reduction_parameter();

        b_error( 0 ) = k_cor * ( m_anc_B_wcs( 0 ) - m_anc_A_wcs( 0 ) );
        b_error( 1 ) = k_cor * ( m_anc_B_wcs( 1 ) - m_anc_A_wcs( 1 ) );
        b_error( 2 ) = k_cor * ( m_anc_B_wcs( 2 ) - m_anc_A_wcs( 2 ) );
        b_error( 3 ) = k_cor * ( -m_s_A_wcs * m_s_B_wcs );
        if ( m_limit_offset1 )
          m_limit1->get_stabilization_term( b_error, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_stabilization_term( b_error, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_stabilization_term( b_error, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_stabilization_term( b_error, m_motor_offset2 );
      }

      void get_low_limits( vector_range & lo ) const
      {
        assert( lo.size() == m_rows || !"UniversalJoint::get_low_limits(): incorrect dimensions");

        lo(0) = OpenTissue::math::detail::lowest<real_type>();
        lo(1) = OpenTissue::math::detail::lowest<real_type>();
        lo(2) = OpenTissue::math::detail::lowest<real_type>();
        lo(3) = OpenTissue::math::detail::lowest<real_type>();
        if ( m_limit_offset1 )
          m_limit1->get_low_limits( lo, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_low_limits( lo, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_low_limits( lo, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_low_limits( lo, m_motor_offset2 );
      }

      void get_high_limits( vector_range & hi ) const
      {
        assert( hi.size() == m_rows || !"UniversalJoint::get_high_limits(): incorrect dimensions");

        hi(0) = OpenTissue::math::detail::highest<real_type>();
        hi(1) = OpenTissue::math::detail::highest<real_type>();
        hi(2) = OpenTissue::math::detail::highest<real_type>();
        hi(3) = OpenTissue::math::detail::highest<real_type>();
        if ( m_limit_offset1 )
          m_limit1->get_high_limits( hi, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_high_limits( hi, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_high_limits( hi, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_high_limits( hi, m_motor_offset2 );
      }

      void get_dependency_indices( idx_vector_range & dep ) const
      {
        assert( dep.size() == m_rows || !"UniversalJoint::get_dependency_indices(): incorrect dimensions");
        for ( size_type i = 0;i < m_rows;++i )
          dep( i ) = OpenTissue::math::detail::highest<size_type>();
      }

      void get_dependency_factors( vector_range & factors ) const
      {
        assert( factors.size() == m_rows || !"UniversalJoint::get_dependency_factors(): incorrect dimensions");
        for ( size_type i = 0;i < m_rows;++i )
          factors( i ) = value_traits::zero();
      }

      void set_regularization( vector_range const & gamma )
      {
        assert( gamma.size() == m_rows || !"UniversalJoint::set_regularization(): incorrect dimensions");
        math_policy::resize( m_gamma, 4 );
        for ( size_type i = 0;i < 4;++i )
        {
          assert( gamma( i ) <= value_traits::one()  || !"UniversalJoint::set_regularization(): gamma was greater than 1");
          assert( gamma( i ) >= value_traits::zero() || !"UniversalJoint::set_regularization(): gamma was less than 0");
          m_gamma( i ) = gamma( i );
        }
        if ( m_limit_offset1 )
          m_limit1->set_regularization( gamma, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->set_regularization( gamma, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->set_regularization( gamma, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->set_regularization( gamma, m_motor_offset2 );
      }

      void get_regularization( vector_range & gamma ) const
      {
        assert( gamma.size() == m_rows || !"UniversalJoint::get_regularization(): incorrect dimensions");
        if ( m_gamma.size() == 0 )
        {
          for ( size_type i = 0;i < 4;++i )
            gamma( i ) = value_traits::zero();
        }
        else
        {
          for ( size_type i = 0;i < 4;++i )
            gamma( i ) = m_gamma( i );
        }
        if ( m_limit_offset1 )
          m_limit1->get_regularization( gamma, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_regularization( gamma, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_regularization( gamma, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_regularization( gamma, m_motor_offset2 );
      }

      void set_solution( vector_range const & solution )
      {
        assert( solution.size() == m_rows || !"UniversalJoint::set_solution(): incorrect dimensions");
        math_policy::resize( m_solution, 4 );
        for ( size_type i = 0;i < 4;++i )
          m_solution( i ) = solution( i );
        if ( m_limit_offset1 )
          m_limit1->set_solution( solution, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->set_solution( solution, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->set_solution( solution, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->set_solution( solution, m_motor_offset2 );
      }

      void get_solution( vector_range & solution ) const
      {
        assert( solution.size() == m_rows || !"UniversalJoint::get_solution(): incorrect dimensions");
        if ( m_solution.size() == 0 )
        {
          for ( size_type i = 0;i < 4;++i )
            solution( i ) = value_traits::zero();
        }
        else
        {
          for ( size_type i = 0;i < 4;++i )
            solution( i ) = m_solution( i );
        }
        if ( m_limit_offset1 )
          m_limit1->get_solution( solution, m_limit_offset1 );
        if ( m_motor_offset1 )
          m_motor1->get_solution( solution, m_motor_offset1 );
        if ( m_limit_offset2 )
          m_limit2->get_solution( solution, m_limit_offset2 );
        if ( m_motor_offset2 )
          m_motor2->get_solution( solution, m_motor_offset2 );
      }

      void calibration()
      {
        using std::fabs;

        assert( this->m_socketA || !"UniversalJoint::calibration(): socket A was null");
        assert( this->m_socketB || !"UniversalJoint::calibration(): socket B was null");

        quaternion_type Q_A, Q_B;
        this->m_socketA->get_body() ->get_orientation( Q_A );
        this->m_socketB->get_body() ->get_orientation( Q_B );
        //---  Q_initial : BF_A -> BF_B
        m_Q_initial = conj( Q_B ) % Q_A;
        m_Q_initial_conj = conj( m_Q_initial );


        vector3_type s_A_wcs = this->m_socketA->get_joint_axis_world();
        vector3_type s_B_wcs = this->m_socketB->get_joint_axis_world();
        //--- verify that joint was set up correctly.
        assert( fabs( s_A_wcs * s_B_wcs ) < 10e-7 || !"UniversalJoint::calibration(): axes where orthogonal"); 
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_UNIVERSAL_JOINT_H
#endif
