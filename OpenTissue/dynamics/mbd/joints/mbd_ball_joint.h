#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_BALL_JOINT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_BALL_JOINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_joint_interface.h>
#include <OpenTissue/dynamics/mbd/limits/mbd_reach_cone.h>
#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/core/math/math_is_number.h>

namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types >
    class BallJoint 
      : public JointInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy                    math_policy;
      typedef typename mbd_types::math_policy::value_traits      value_traits;
      typedef typename mbd_types::math_policy::index_type        size_type;
      typedef typename mbd_types::math_policy::real_type         real_type;
      typedef typename mbd_types::math_policy::vector_range      vector_range;
      typedef typename mbd_types::math_policy::vector3_type      vector3_type;
      typedef typename mbd_types::math_policy::matrix3x3_type    matrix3x3_type;
      typedef typename mbd_types::math_policy::quaternion_type   quaternion_type;
      typedef typename mbd_types::math_policy::idx_vector_range  idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range      matrix_range;
      typedef typename mbd_types::math_policy::vector_type       vector_type;
      typedef          ReachCone<mbd_types>                 reach_cone_type;

    protected:

      matrix3x3_type   m_star_anc_A;      ///< The cross product operator of the anchor point wrt. to body A rotated (but not translated) into WCS.
      matrix3x3_type   m_star_anc_B;      ///< The cross product operator of the anchor point wrt. to body B rotated (but not translated) into WCS.
      vector3_type     m_anc_A_wcs;       ///< The anchor point wrt. to body A in WCS.
      vector3_type     m_anc_B_wcs;       ///< The anchor point wrt. to body B in WCS.
      quaternion_type  m_Q_initial;       ///< The initial relative orientation between the two bodies (wrt. WCS) i.e. the rotation that aligns body A with body B.
      quaternion_type  m_Q_initial_conj;  ///< The conjugate of Q_initial.

      size_type        m_rows;            ///< Number of jacobian rows.
      vector_type      m_gamma;           ///< Local vector of constraint force mixing terms.
      vector_type      m_solution;        ///< Local solution vector, ie. vector of lagrange multipliers.

      reach_cone_type  *m_reach_cone;     ///< Reach cone.
      size_type         m_reach_offset;   ///< Jacobian row offset for reach cone constraints.

    public:

      BallJoint()
        : m_rows(0)
        , m_reach_cone(0)
        , m_reach_offset(0)
      {}

      virtual ~BallJoint() {}

    public:

      void evaluate()
      {
        m_rows = 0;
        if(!(this->m_socketA && this->m_socketB))//--- joint_type must be disconnected nothing to evaluate
          return;

        m_rows = 3;

        quaternion_type Q_a,Q_b;

        this->m_socketA->get_body()->get_orientation(Q_a);
        m_star_anc_A = star( Q_a.rotate(  this->m_socketA->get_anchor_local()) );

        assert(is_number(m_star_anc_A(0,0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(0,1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(0,2)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(1,0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(1,1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(1,2)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(2,0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(2,1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_A(2,2)) || !"BallJoint::evaluate(): not a number");

        this->m_socketB->get_body()->get_orientation(Q_b);
        m_star_anc_B = star( Q_b.rotate(  this->m_socketB->get_anchor_local()) );

        assert(is_number(m_star_anc_B(0,0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(0,1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(0,2)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(1,0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(1,1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(1,2)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(2,0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(2,1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_star_anc_B(2,2)) || !"BallJoint::evaluate(): not a number");

        m_anc_A_wcs = this->m_socketA->get_anchor_world();
        m_anc_B_wcs = this->m_socketB->get_anchor_world();

        assert(is_number(m_anc_A_wcs(0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_anc_A_wcs(1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_anc_A_wcs(2)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_anc_B_wcs(0)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_anc_B_wcs(1)) || !"BallJoint::evaluate(): not a number");
        assert(is_number(m_anc_B_wcs(2)) || !"BallJoint::evaluate(): not a number");

        m_reach_offset = 0;
        if(m_reach_cone)
        {
          quaternion_type   J_a = this->m_socketA->get_joint_frame().Q();
          quaternion_type   J_b = this->m_socketB->get_joint_frame().Q();
          quaternion_type     Q = conj(J_a) % conj(Q_a) % Q_b % J_b;


          assert(is_number(Q.s())    || !"BallJoint::evaluate(): not a number");
          assert(is_number(Q.v()(0)) || !"BallJoint::evaluate(): not a number");
          assert(is_number(Q.v()(1)) || !"BallJoint::evaluate(): not a number");
          assert(is_number(Q.v()(2)) || !"BallJoint::evaluate(): not a number");

          vector3_type s_local  = Q.rotate( this->m_socketB->get_joint_axis_local() );
          vector3_type s_wcs    = get_axis_world();

          assert(is_number(s_wcs(0)) || !"BallJoint::evaluate(): not a number");
          assert(is_number(s_wcs(1)) || !"BallJoint::evaluate(): not a number");
          assert(is_number(s_wcs(2)) || !"BallJoint::evaluate(): not a number");

          m_reach_cone->evaluate(Q,s_wcs,s_local);
          if(m_reach_cone->is_active())
          {
            m_reach_offset = m_rows;
            m_rows += m_reach_cone->get_number_of_jacobian_rows();
          }
        }
      }


      size_type get_number_of_jacobian_rows() const {return m_rows;}

      void get_linear_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"BallJoint::get_linear_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"BallJoint::get_linear_jacobian_A(): incorrect dimensions");
        J(0,0) = value_traits::one();  J(0,1) = value_traits::zero(); J(0,2) = value_traits::zero();
        J(1,0) = value_traits::zero(); J(1,1) = value_traits::one();  J(1,2) = value_traits::zero();
        J(2,0) = value_traits::zero(); J(2,1) = value_traits::zero(); J(2,2) = value_traits::one();
        if(m_reach_offset)
          m_reach_cone->get_linear_jacobian_A(J,m_reach_offset);
      }

      void get_linear_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"BallJoint::get_linear_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"BallJoint::get_linear_jacobian_B(): incorrect dimensions");
        J(0,0) = -value_traits::one();  J(0,1) =  value_traits::zero(); J(0,2) =  value_traits::zero();
        J(1,0) =  value_traits::zero(); J(1,1) = -value_traits::one();  J(1,2) =  value_traits::zero();
        J(2,0) =  value_traits::zero(); J(2,1) =  value_traits::zero(); J(2,2) = -value_traits::one();
        if(m_reach_offset)
          m_reach_cone->get_linear_jacobian_B(J,m_reach_offset);
      }

      void get_angular_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"BallJoint::get_angular_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"BallJoint::get_angular_jacobian_A(): incorrect dimensions");
        J(0,0) = -m_star_anc_A(0,0); J(0,1) = -m_star_anc_A(0,1); J(0,2) = -m_star_anc_A(0,2);
        J(1,0) = -m_star_anc_A(1,0); J(1,1) = -m_star_anc_A(1,1); J(1,2) = -m_star_anc_A(1,2);
        J(2,0) = -m_star_anc_A(2,0); J(2,1) = -m_star_anc_A(2,1); J(2,2) = -m_star_anc_A(2,2);
        if(m_reach_offset)
          m_reach_cone->get_angular_jacobian_A(J,m_reach_offset);
      }

      void get_angular_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"BallJoint::get_angular_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"BallJoint::get_angular_jacobian_B(): incorrect dimensions");
        J(0,0) = m_star_anc_B(0,0); J(0,1) = m_star_anc_B(0,1); J(0,2) = m_star_anc_B(0,2);
        J(1,0) = m_star_anc_B(1,0); J(1,1) = m_star_anc_B(1,1); J(1,2) = m_star_anc_B(1,2);
        J(2,0) = m_star_anc_B(2,0); J(2,1) = m_star_anc_B(2,1); J(2,2) = m_star_anc_B(2,2);
        if(m_reach_offset)
          m_reach_cone->get_angular_jacobian_B(J,m_reach_offset);
      }

      void get_stabilization_term(vector_range & b_error)const
      {
        assert(b_error.size()==m_rows || !"BallJoint::get_stabilization_term(): incorrect dimensions");
        real_type k_cor = this->get_frames_per_second()*this->get_error_reduction_parameter();

        assert(is_number(k_cor) || !"BallJoint::get_stabilization_term(): not a number");

        b_error(0) = k_cor*(m_anc_B_wcs(0) - m_anc_A_wcs(0));
        b_error(1) = k_cor*(m_anc_B_wcs(1) - m_anc_A_wcs(1));
        b_error(2) = k_cor*(m_anc_B_wcs(2) - m_anc_A_wcs(2));

        assert(is_number(b_error(0)) || !"BallJoint::get_stabilization_term(): not a number");
        assert(is_number(b_error(1)) || !"BallJoint::get_stabilization_term(): not a number");
        assert(is_number(b_error(2)) || !"BallJoint::get_stabilization_term(): not a number");

        if(m_reach_offset)
          m_reach_cone->get_stabilization_term(b_error,m_reach_offset);
      }

      void get_low_limits(vector_range & lo)const
      {
        assert(lo.size()==m_rows || !"BallJoint::get_low_limits(): incorrect dimension");
        lo(0) = OpenTissue::math::detail::lowest<real_type>();
        lo(1) = OpenTissue::math::detail::lowest<real_type>();
        lo(2) = OpenTissue::math::detail::lowest<real_type>();

        assert(is_number(lo(0)) || !"BallJoint::get_low_limits(): not a number");
        assert(is_number(lo(1)) || !"BallJoint::get_low_limits(): not a number");
        assert(is_number(lo(2)) || !"BallJoint::get_low_limits(): not a number");

        if(m_reach_offset)
          m_reach_cone->get_low_limits(lo,m_reach_offset);
      }

      void get_high_limits(vector_range & hi)const
      {
        assert(hi.size()==m_rows || !"BallJoint::get_high_limits(): incorrect dimension");
        hi(0) = OpenTissue::math::detail::highest<real_type>();
        hi(1) = OpenTissue::math::detail::highest<real_type>();
        hi(2) = OpenTissue::math::detail::highest<real_type>();

        assert(is_number(hi(0)) || !"BallJoint::get_high_limits(): not a number");
        assert(is_number(hi(1)) || !"BallJoint::get_high_limits(): not a number");
        assert(is_number(hi(2)) || !"BallJoint::get_high_limits(): not a number");

        if(m_reach_offset)
          m_reach_cone->get_high_limits(hi,m_reach_offset);
      }

      void get_dependency_indices(idx_vector_range & dep)const
      {
        assert(dep.size()==m_rows || !"BallJoint::get_dependency_indices(): incorrect dimension");
        for(size_type i=0;i<m_rows;++i)
          dep(i) = OpenTissue::math::detail::highest<size_type>();
      }

      void get_dependency_factors(vector_range & factors)const
      {
        assert(factors.size()==m_rows || !"BallJoint::get_dependency_factors(): incorrect dimension");
        for(size_type i=0;i<m_rows;++i)
          factors(i) = value_traits::zero();
      }

      void set_regularization(vector_range const & gamma)
      {
        assert(gamma.size()==m_rows || !"BallJoint::set_regularization(): incorrect dimension");
        math_policy::resize( m_gamma, 3);
        for(size_type i=0;i<3;++i)
        {
          assert(gamma(i)<=value_traits::one()  || !"BallJoint::set_regularization(): gamma was greather than 1");
          assert(gamma(i)>=value_traits::zero() || !"BallJoint::set_regularization(): gamma was less than 0");
          m_gamma(i) = gamma(i);
        }

        assert(is_number(gamma(0)) || !"BallJoint::set_regularization(): not a number");
        assert(is_number(gamma(1)) || !"BallJoint::set_regularization(): not a number");
        assert(is_number(gamma(2)) || !"BallJoint::set_regularization(): not a number");

        if(m_reach_offset)
          m_reach_cone->set_regularization(gamma,m_reach_offset);
      }

      void get_regularization(vector_range & gamma)const
      {
        assert(gamma.size()==m_rows|| !"BallJoint::get_regularization(): incorrect dimension");
        if(m_gamma.size()==0)
        {
          for(size_type i=0;i<3;++i)
            gamma(i) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<3;++i)
            gamma(i) = m_gamma(i);
        }

        assert(is_number(gamma(0)) || !"BallJoint::get_regularization(): not a number");
        assert(is_number(gamma(1)) || !"BallJoint::get_regularization(): not a number");
        assert(is_number(gamma(2)) || !"BallJoint::get_regularization(): not a number");

        if(m_reach_offset)
          m_reach_cone->get_regularization(gamma,m_reach_offset);
      }

      void set_solution(vector_range const & solution)
      {
        assert(solution.size()==m_rows || !"BallJoint::set_solution(): incorrect dimension");

        math_policy::resize( m_solution, 5);
        for(size_type i=0;i<3;++i)
          m_solution(i) = solution(i);

        assert(is_number(solution(0)) || !"BallJoint::set_solution(): not a number");
        assert(is_number(solution(1)) || !"BallJoint::set_solution(): not a number");
        assert(is_number(solution(2)) || !"BallJoint::set_solution(): not a number");

        if(m_reach_offset)
          m_reach_cone->set_solution(solution,m_reach_offset);
      }

      void get_solution(vector_range & solution)const
      {
        assert(solution.size()==m_rows || !"BallJoint::get_solution(): incorrect dimension");

        if(m_solution.size()==0)
        {
          for(size_type i=0;i<3;++i)
            solution(i) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<3;++i)
            solution(i) = m_solution(i);
        }

        assert(is_number(solution(0)) || !"BallJoint::get_solution(): not a number");
        assert(is_number(solution(1)) || !"BallJoint::get_solution(): not a number");
        assert(is_number(solution(2)) || !"BallJoint::get_solution(): not a number");

        if(m_reach_offset)
          m_reach_cone->get_solution(solution,m_reach_offset);

      }

      void calibration()
      {
        assert(this->m_socketA|| !"BallJoint::calibration(): socket A was null");
        assert(this->m_socketB|| !"BallJoint::calibration(): socket B was null");

        quaternion_type Q_A,Q_B;
        this->m_socketA->get_body()->get_orientation(Q_A);
        this->m_socketB->get_body()->get_orientation(Q_B);


        assert(is_number(Q_A.s()) || !"BallJoint::calibration(): not a number");
        assert(is_number(Q_A.v()(0)) || !"BallJoint::calibration(): not a number");
        assert(is_number(Q_A.v()(1)) || !"BallJoint::calibration(): not a number");
        assert(is_number(Q_A.v()(2)) || !"BallJoint::calibration(): not a number");

        assert(is_number(Q_B.s()) || !"BallJoint::calibration(): not a number");
        assert(is_number(Q_B.v()(0)) || !"BallJoint::calibration(): not a number");
        assert(is_number(Q_B.v()(1)) || !"BallJoint::calibration(): not a number");
        assert(is_number(Q_B.v()(2)) || !"BallJoint::calibration(): not a number");

        //---
        //---  Q_A :  BF_A -> WCS
        //---  Q_B :  BF_B -> WCS   => Q_B^* : WCS -> BF_B
        //---
        //---  Q_initial : BF_A -> BF_B  => Q_initial = Q_B^*  Q_A
        //---
        m_Q_initial = conj(Q_B) % Q_A;
        m_Q_initial_conj = conj(m_Q_initial);

        assert(is_number(m_Q_initial_conj.s()) || !"BallJoint::calibration(): not a number");
        assert(is_number(m_Q_initial_conj.v()(0)) || !"BallJoint::calibration(): not a number");
        assert(is_number(m_Q_initial_conj.v()(1)) || !"BallJoint::calibration(): not a number");
        assert(is_number(m_Q_initial_conj.v()(2)) || !"BallJoint::calibration(): not a number");
      }

      vector3_type get_axis_world() const 
      {
        assert(is_number(this->m_socketA->get_joint_axis_world()(0)) || !"BallJoint::get_axis_world(): not a number");
        assert(is_number(this->m_socketA->get_joint_axis_world()(1)) || !"BallJoint::get_axis_world(): not a number");
        assert(is_number(this->m_socketA->get_joint_axis_world()(2)) || !"BallJoint::get_axis_world(): not a number");

        return this->m_socketA->get_joint_axis_world(); 
      }

      void set_reach_cone(reach_cone_type const & cone) {   m_reach_cone = const_cast<reach_cone_type*>(&cone); }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_BALL_JOINT_H
#endif
