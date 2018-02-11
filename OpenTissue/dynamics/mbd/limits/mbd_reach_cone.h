#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_REACH_CONE_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_REACH_CONE_H
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

    /**
    * A Reach Cone Limit.
    *
    * The reach cone modelling idea was inspired from the work
    *
    *  @article{wilhelms.van_gelder.01,
    *     author = {Jane Wilhelms and Allen Van Gelder},
    *     title = {Fast and easy reach-cone joint limits},
    *     journal = {J. Graph. Tools},
    *     volume = {6},
    *     number = {2},
    *     year = {2001},
    *     issn = {1086-7651},
    *     pages = {27--41},
    *     publisher = {A. K. Peters, Ltd.},
    *     address = {Natick, MA, USA},
    *  }
    *
    * See http://jgt.akpeters.com/papers/WilhelmsVanGelder01/ also have a look at 
    *
    * @article{WilhelmsVanGelder01,
    *   author = "Jane Wilhelms and Allen Van Gelder",
    *   title = "Fast and Easy Reach-Cone joint_type Limits",
    *   journal = "journal of graphics tools",
    *   volume = "6",
    *   number = "2",
    *   pages = "27-41",
    *   year = "2001",
    *   }
    *
    * Our modelling method is different, but the ideas are similar. In this implementation
    * a reach cone is simply a collection of angular joint limits.
    *
    * Initially the joint limits are specified by a face-plane normal of each side
    * of the cone. This is simply another way of specifying an angular limit
    * on a ``rotational'' joint type. When the method ReachCone::evaluate
    * is invoked active cone constraints are converted into the typical representation
    * of an angular joint limit (rotation axis and error angle).
    *
    * During simulation the cone sides are evaluated (see ReachCone::evaluate) to
    * see whether they are active or not. All active cone side constraints are kept in
    * a local member of the class (see ReachCone::m_contraints) and used to create
    * the rows of the constraint jacobian matrix (see ReachCone::get_angular_jacobian_X)
    * and error terms (see ReachCone::get_stabilization_term).
    *
    *
    * IMPORTANT: Remember that the joint axis of a joint wrt. a given
    * body is specified by the third axis of the joint frame (see
    * JointSocket::get_joint_axis_local) attached to the given body.
    *
    * A joint in its zero (rest) position has the joint frames of body A and B
    * in complete alignment. Thus in the world frame the z-axis of the joint frame
    * on body B is aligned with the z-axis of the joint frame on body A.
    *
    * Thus when specifying cone planes, these are described such that the
    * joint axis wrt. body A (i.e. z-axis of joint frame on body A) is inside
    * the cone. During simulation it is tested if the current joint axis wrt.
    * body B has tipped outside the cone which is kept fixed wrt. to the joint
    * frame on body A.
    */
    template< typename mbd_types  >
    class ReachCone
      : public SubConstraintInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy                       math_policy;
      typedef typename mbd_types::math_policy::real_type            real_type;
      typedef typename mbd_types::math_policy::size_type            size_type;
      typedef typename mbd_types::math_policy::value_traits         value_traits;
      typedef typename mbd_types::math_policy::vector3_type         vector3_type;
      typedef typename mbd_types::math_policy::vector_range         vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range     idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range         matrix_range;
      typedef typename mbd_types::math_policy::vector_type          vector_type;
      typedef typename mbd_types::math_policy::quaternion_type      quaternion_type;

    protected:

      struct cone_constraint_type
      {
        real_type       m_theta_err;    ///< Angle error.
        vector3_type    m_u;            ///< Constraint axis.
      };

      typedef typename std::vector<cone_constraint_type>     constraint_container;
      typedef typename std::vector<vector3_type>             normal_container;

      constraint_container  m_contraints;       ///< Collection of active cone constraints. Each constraint is basically
                                                ///< just an angular joint limit.

      normal_container      m_cone_normals;     ///< A side of the cone is described as the plane
                                                ///< containing the joint position (the pivot point
                                                ///< of the rotation) and an inward unit plane normal (normal points to
                                                ///< the legal reach region).
                                                ///< It is implictly assumed that the ``convex'' inside
                                                ///< region of the planes correspond to the reach cone.

      size_type          m_cone_sides;       ///< The number of cone sides.
      size_type          m_rows;             ///< Number of jacobian rows.
      vector_type        m_gamma;            ///< Local vector of constraint force mixing terms.
      vector_type        m_solution;         ///< Local solution vector, ie. vector of lagrange multipliers.

    public:

      ReachCone()
        : m_cone_sides(0)
        , m_rows(0)
        , m_gamma(0)
        , m_solution(0)
      {}

      virtual ~ReachCone(){}

    public:

      /**
      * Add Cone.
      *
      * @param n_local     Inward cone face normal in local joint frame of body A.
      */
      void add_cone(vector3_type const & n_local)
      {
        vector3_type n_unit = normalize(n_local);
        //--- Test whether n_unit and the joint axis from the joint
        //--- frame on body A points in the same direction. I.e. whether
        //--- their dot product is non-negative.
        //--- If the dot product is negative then the user
        //--- is trying to setup and inconsistent reach cone.
        //---
        //--- But, in the local frame of the joint frame wrt. body A, the
        //--- joint axis is always (0,0,1) by convention. So this implies
        //--- that the z-coordinate of n_unit must be non-negative!
        if(n_unit(2)<value_traits::zero())
        {
          std::cout << "ReachCone::add_cone(): Inconsistent cone side, dropping it" << std::endl;
          return;
        }      
        m_cone_normals.push_back(n_unit);
        m_contraints.push_back(cone_constraint_type());
        ++m_cone_sides;
      }

      /**
      * Evaluate Constraints.
      *
      * This method loops over all the sides of the cone. For each side of the
      * cone the method evaluates whether the joint orientation violates the
      * corresponding cone plane. If this is the case an corresponding angular
      * joint limit is created.
      *
      * @param Q         Rotation from local joint frame of body A to WCS. That is something like
      *
      *                      bodyA->get_orientation(Q_a);
      *                      Q = Q_a % socketA->get_joint_frame().Q()
      *
      * @param s_wcs     joint_type axis of body A in wcs
      * @param s_local   joint_type axis of body B in local joint frame of body A.
      *
      *                     JF_A = socketA->get_joint_frame().Q()
      *                     JF_B = socketB->get_joint_frame().Q()
      *                     bodyA->get_orientation(BF_A);
      *                     bodyB->get_orientation(BF_B);
      *                     Q = conj(JF_A) conj(BF_A) BF_B JF_B
      *                     s_local = Q.rotate(s_b);
      *
      */
      void evaluate(
        quaternion_type const & Q,
        vector3_type const & s_wcs,
        vector3_type const & s_local
        )
      {
        //std::cout << "ReachCone::evalute(): invoked " << std::endl;
        m_rows = 0;
        for( size_type i=0;i<m_cone_sides;++i)
        {
          //--- Compute the angle between the joint axis and the cone plane normal.
          vector3_type n_local = m_cone_normals[i];

          //---  Given two vectors n and s then the angle between them is given as
          //---
          //---      cos theta = n * s / |n| |s|
          //---  
          //--- If both vectors are unit vectors and we apply the small angle
          //--- approximation then we have
          //---
          //---      theta = n*s
          //---
          real_type theta_err = n_local * s_local;
          //--- If the angle violates the plane ``slope'' then create a angular joint limit.

          //--- The normal points to the inside region of the cone, thus we need to
          //--- test whether the s_local vector have tipped to the other side of the
          //--- cone plane. That is whether the cone face normal and the joint axis
          //--- wrt. body B points in opposite directions.
          if(theta_err<value_traits::zero()) 
          {
            // TODO KE 2006-08-06: If violation is greater than pi/2 radians
            // then we will not see the pi/2 error in the angle!

            m_contraints[m_rows].m_theta_err = theta_err;
            //std::cout << "\ttheta error = " << theta_err << std::endl;

            //--- Compute the current joint angle limit rotation axis. That is
            //--- the axis we need to rotate around to make the joint violation
            //--- disappear.
            vector3_type n_wcs = Q.rotate(n_local);
            m_contraints[m_rows].m_u = cross(n_wcs , s_wcs);
            ++m_rows;
          }
        }
      }

      size_type get_number_of_jacobian_rows() const {return m_rows;}

      void get_linear_jacobian_A(matrix_range & J,size_type const & offset)const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          J(row,0) = value_traits::zero();
          J(row,1) = value_traits::zero();
          J(row,2) = value_traits::zero();
        }
      }

      void get_linear_jacobian_B(matrix_range & J,size_type const & offset)const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          J(row,0) = value_traits::zero();
          J(row,1) = value_traits::zero();
          J(row,2) = value_traits::zero();
        }
      }

      void get_angular_jacobian_A(matrix_range & J,size_type const & offset)const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          J(row,0) = -m_contraints[i].m_u(0);
          J(row,1) = -m_contraints[i].m_u(1);
          J(row,2) = -m_contraints[i].m_u(2);
        }
      }

      void get_angular_jacobian_B(matrix_range & J,size_type const & offset)const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          J(row,0) = m_contraints[i].m_u(0);
          J(row,1) = m_contraints[i].m_u(1);
          J(row,2) = m_contraints[i].m_u(2);
        }
      }

      void get_stabilization_term(vector_range & b_error,size_type const & offset)const
      {
        real_type k_cor = this->get_frames_per_second()*this->get_error_reduction_parameter();
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          b_error(row) = - k_cor*m_contraints[i].m_theta_err;
        }
      }

      void get_low_limits(vector_range & lo,size_type const & offset)const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          lo(row) = value_traits::zero();
        }
      }

      void get_high_limits(vector_range & hi,size_type const & offset)const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          hi(row) = OpenTissue::math::detail::highest<real_type>();
        }
      }

      void get_dependency_indices(idx_vector_range & dep,size_type const & offset) const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          dep(row) = OpenTissue::math::detail::highest<size_type>();
        }
      }

      void get_dependency_factors(vector_range & factors,size_type const & offset) const
      {
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          factors(row) = value_traits::zero();
        }
      }

      void set_regularization(vector_range const & gamma,size_type const & offset)
      {
        math_policy::resize( m_gamma, m_rows);
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
        {
          assert(gamma(row)<=1 || !"ReachCone::set_regularization(): gamma value out of range");
          assert(gamma(row)>=0 || !"ReachCone::set_regularization(): gamma value out of range");
          m_gamma(i) = gamma(row);
        }
      }

      void get_regularization(vector_range & gamma,size_type const & offset)const
      {
        size_type row = offset;
        if(m_gamma.size()==0)
        {
          for(size_type i=0;i<m_rows;++i,++row)
            gamma(row) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<m_rows;++i,++row)
            gamma(row) = m_gamma(i);
        }
      }

      void set_solution(vector_range const & solution,size_type const & offset)
      {
        assert(solution.size()==m_rows || !"ReachCone::set_solution(): solution array size did not match number of active cone constraints");
        math_policy::resize( m_solution, m_rows);
        size_type row = offset;
        for(size_type i=0;i<m_rows;++i,++row)
          m_solution(i) = solution(row);
      }

      void get_solution(vector_range & solution,size_type const & offset)const
      {
        size_type row = offset;
        if(m_solution.size()==0)
        {
          for(size_type i=0;i<m_rows;++i,++row)
            solution(row) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<m_rows;++i,++row)
            solution(row) = m_solution(i);
        }
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_LIMITS_MBD_REACH_CONE_H
#endif
