#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BONE_TRAITS_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BONE_TRAITS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/inverse/inverse_box_limits.h>
#include <OpenTissue/kinematics/inverse/inverse_sinus_cone.h>
#include <OpenTissue/kinematics/inverse/inverse_reach_cone.h>

#include <cassert>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
      
      /**
       * Default Bone Traits for Inverse Kinematics.
       * This class provides functionality to handle constrained inverse kinematics.
       *
       * @warning If end-users wants to extend with their own bone traits
       *          then their bone traits must comply with a similar interface
       *          as this bone traits class has.
       *
       * @tparam  base_bone_traits     A base bone traits. This could for instance be the default
       *                               bone traits from the skeleton library. This traits class must
       *                               define the types of transforms and different ways of manipulating
       *                               and doing calculations with these types. Thus this bone trait
       *                               class does not care about what kind of coordinate representation
       *                               one wants to use for the skeleton/bone transformations.
       */
      template<typename base_bone_traits >
      class BoneTraits 
        : public base_bone_traits
        {
        public:
          
          friend class ACCESSOR;
          
          typedef          BoxLimits< base_bone_traits >                  box_limits_type;
          typedef          SinusCone< base_bone_traits >                  sinus_cone_type;
          typedef          ReachCone< base_bone_traits >                  reach_cone_type;
          
          typedef enum {box_type, sinus_type, reach_type}                 limits_model_type;
          
          
          typedef typename base_bone_traits::transform_type               transform_type;
          typedef typename base_bone_traits::vector3_type                 vector3_type;
          typedef typename base_bone_traits::math_types::real_type        real_type;
          typedef typename base_bone_traits::math_types::value_traits     value_traits;
          
          typedef enum {hinge_type, slider_type, ball_type}               joint_type;
          
        protected:
          
          joint_type            m_type;              ///< The joint type that this bone is connected to its parent bone with.
          real_type             m_theta[6];          ///< Joint parameter values.
          vector3_type          m_u;                 ///< Unit joint axis, given as a constant vector in parent frame.
          
          box_limits_type       m_box_limits;        ///< Box limits.
          sinus_cone_type       m_sinus_cone;        ///< A Sinus Cone.
          reach_cone_type       m_reach_cone;        ///< A Reach Cone.
          
          limits_model_type     m_limits_model;      ///< Indication of the type of joint limit model that is used for the bone. The default value is box limits.
          
        protected:
          
          /**
           * Get Joint Parameter Value.
           *
           * @param i     The index of the joint paramter value that is wanted.
           * @return      A reference to the corresponding joint parameter value.
           */
          real_type const & theta(size_t const & i) const 
          {
            assert( i < this->active_dofs() || !"theta(): invalid index value");
            return m_theta[i];
          }
          
          /**
           * Get Joint Parameter Value.
           *
           * @param i     The index of the joint parameter value that is wanted.
           * @return      A reference to the corresponding joint parameter value.
           */
          real_type      & theta(size_t const & i)
          {
            assert( i < this->active_dofs() || !"theta(): invalid index value");
            return m_theta[i];
          }
          
        public:
          
          /**
           * Get Box Limits.
           * The box limits object can be used to define the actual box limits of
           * the bone. By default rather loose box limits are set up for the bone.
           *
           * @see BoxLimits.
           *
           * @return   A reference to the box limits object of the bone.
           */
          box_limits_type const & box_limits() const { return m_box_limits; }
          box_limits_type       & box_limits()       { return m_box_limits; }
          
          /**
           * Get Sinus Cone.
           *
           * @see SinusCone.
           *
           * @return   A reference to the sinus cone object of the bone.
           */
          sinus_cone_type const & sinus_cone() const { return m_sinus_cone; }
          sinus_cone_type       & sinus_cone()       { return m_sinus_cone; }
          
          /**
           * Get Reach Cone.
           *
           * @see ReachCone.
           *
           * @return   A reference to the reach cone object of the bone.
           */
          reach_cone_type const & reach_cone() const { return m_reach_cone; }
          reach_cone_type       & reach_cone()       { return m_reach_cone; }
          
          /**
           * Get Limits Model Type.
           * This method should be used to query the limits model type and for
           * changing the limits model used of the bone.
           *
           * @return   A reference to the limits model type.
           */
          
          limits_model_type const & limits_model() const { return m_limits_model; }
          limits_model_type       & limits_model()       { return m_limits_model; }
          
        public:
          
          BoneTraits()
          : m_type(ball_type)
          , m_u( value_traits::zero(), value_traits::zero(), value_traits::one() )
          , m_box_limits()
          , m_sinus_cone()
          , m_reach_cone()
          , m_limits_model( box_type )
          {
            for(size_t i=0;i<6;++i)
              m_theta[i] = value_traits::zero();
          }
          
          /** 
           * Get Joint Type.
           *
           * @return   A reference to the joint type of this bone.
           */
          joint_type const & type() const  { return this->m_type; }
          joint_type       & type()        { return this->m_type; }
          
          /**
           * Get Joint Axis.
           *
           * @return   A reference to the joint axis. This is used
           *           for specifying the motion of slider and revolute
           *           joint types. It is un-needed for ball type joints.
           */
          vector3_type const & u() const  { return this->m_u; }
          vector3_type       & u()        { return this->m_u; }
          
          /**
           * Get number of Active Degrees of Freedon (DOFs).
           * This method computes and returns the number of active degrees
           * of freedom for the bone. This number is related directly to the
           * specified joint type of the bone.
           *
           * @return The number of active degrees of freedom of the bone.
           */
          size_t active_dofs() const 
          {
            if(this->m_type==hinge_type)
              return 1;
            if(this->m_type==slider_type)
              return 1;
            if(this->m_type==ball_type)
              return 3;
            return 0;
          }
          
        protected:
          
          /**
           * Project sub-vector onto joint limits of a Bone.
           *
           * @warning    Note that the joint parameter values are not extracted
           *             from the bone, rather it is assumed that the specified
           *             sub-vector contains the current joint paramter values 
           *             that should be used.
           *
           * @param bone             The bone that should be worked on.
           * @param sub_theta        A vector or vector range that initially holds the current joint
           *                         parameter values of the specified bone and which upon return
           *                         holds the projected values.
           */
          template<typename bone_type, typename vector_range>
          static void joint_limit_projection( bone_type const & bone, vector_range & sub_theta)
          {
            switch( bone.limits_model() )
            {
              case bone_type::box_type:
                box_limits::joint_limit_projection( bone, sub_theta);
                break;
              case bone_type::sinus_type:
                sinus_cone::joint_limit_projection( bone, sub_theta);
                break;
              case bone_type::reach_type:
                reach_cone::joint_limit_projection( bone, sub_theta);
                break;
              default:
                assert(false || !"joint_limit_projection(): Unknown joint limits model"); 
                break;
            }
          }
          
          /**
           * Computes the Jacobian matrix corresponding to this bone
           *
           * @Param bone     The bone for which the Jacobian is to be calculated
           * @Param chain    The corresponding chain for specified bone.
           * @Param J        Upon return this holds the bub-block of the Jacobian
           *                 corresponding to the specified bone and end-effector
           *                 of the corresponding chain.
           */
          template<typename bone_type, typename chain_type, typename matrix_range>
          static void compute_jacobian( bone_type const & bone, chain_type const & chain, matrix_range & J )
          {
            typedef typename bone_type::vector3_type              vector3_type;
            typedef typename bone_type::math_types::value_traits  value_traits;
            typedef typename bone_type::math_types::real_type     real_type;
            
            assert(J.size1() == chain.get_goal_dimension() || !"compute_jacobian() invalid dimension");
            assert(J.size2() == bone.active_dofs()         || !"compute_jacobian() invalid dimension");
            
            // Get tool frame in world coordinate system
            vector3_type p = bone_type::transform_point(  chain.get_end_effector()->absolute(),  chain.p_local() );
            p = p - bone.absolute().T();// new term that was missing and may also be in the article
            vector3_type i = unit(bone_type::transform_vector( chain.get_end_effector()->absolute(),  chain.x_local() ));
            vector3_type j = unit(bone_type::transform_vector( chain.get_end_effector()->absolute(),  chain.y_local() ));
            
            if(bone.type() == bone_type::hinge_type)
            {
              // Get joint axis in world coordinate frame
              vector3_type u = bone.u(); // If bone is root then joint axis is already in world coordinate frame
              if(bone.parent())
              {
                u = bone_type::transform_vector( bone.parent()->absolute(),  bone.u() );
              }
              
              vector3_type const uXp = cross(u, p);
              
              J(0,0) = uXp(0);
              J(1,0) = uXp(1);
              J(2,0) = uXp(2);
              
              if(!chain.only_position() )// we need to do the orientation derivatives also
              {
                vector3_type const uXi = cross(u, i);
                vector3_type const uXj = cross(u, j);
                
                J(3,0) = uXi(0);
                J(4,0) = uXi(1);
                J(5,0) = uXi(2);
                
                J(6,0) = uXj(0);
                J(7,0) = uXj(1);
                J(8,0) = uXj(2);
              }
            }
            else if(bone.type() == bone_type::slider_type)
            {
              // Get joint axis in world coordinate frame
              vector3_type u = bone.u(); // If bone is root then joint axis is already in world coordinate frame
              if(bone.parent())
              {
                u = bone_type::transform_vector( bone.parent()->absolute(),  bone.u() );
              }
              
              J(0,0) = u(0);
              J(1,0) = u(1);
              J(2,0) = u(2);
              
              if(!chain.only_position() )// we need to do the orientation derivatives also
              {
                J(3,0) = value_traits::zero();
                J(4,0) = value_traits::zero();
                J(5,0) = value_traits::zero();
                J(6,0) = value_traits::zero();
                J(7,0) = value_traits::zero();
                J(8,0) = value_traits::zero();
              }
            }
            else if(bone.type() == bone_type::ball_type)
            {
              vector3_type const ii = vector3_type(value_traits::one(),value_traits::zero(),value_traits::zero());
              vector3_type const jj = vector3_type(value_traits::zero(),value_traits::one(),value_traits::zero());
              vector3_type const kk = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one());
              
              // Extract joint angles
              real_type phi   = bone.theta(0);
              real_type psi   = bone.theta(1);
              //real_type theta = bone.theta(2);
              
              // Compute the instantaneous axis of rotation (in parent frame)
              vector3_type u = kk;
              vector3_type v = OpenTissue::math::Rz(phi)*jj;
              vector3_type w = OpenTissue::math::Rz(phi)*OpenTissue::math::Ry(psi)*kk;
              
              // Transform instantaneous rotation axes into world coordinate frame
              if(bone.parent())
              {
                vector3_type tmp = bone_type::transform_vector( bone.parent()->absolute(),  u );
                u = unit(tmp);
                tmp = bone_type::transform_vector( bone.parent()->absolute(),  v );
                v = unit(tmp);
                tmp = bone_type::transform_vector( bone.parent()->absolute(),  w );
                w = unit(tmp);
              }
              
              vector3_type const uXp = cross(u, p);
              vector3_type const vXp = cross(v, p);
              vector3_type const wXp = cross(w, p);
              
              // setup phi part
              J(0,0) = uXp(0);
              J(1,0) = uXp(1);
              J(2,0) = uXp(2);
              
              // setup psi part
              J(0,1) = vXp(0);
              J(1,1) = vXp(1);
              J(2,1) = vXp(2);
              
              // setup theta part
              J(0,2) = wXp(0);
              J(1,2) = wXp(1);
              J(2,2) = wXp(2);
              
              if( ! chain.only_position() ) // we need to do the orientation derivatives also
              {
                vector3_type const uXi = cross(u, i);
                vector3_type const uXj = cross(u, j);
                
                vector3_type const vXi = cross(v, i);
                vector3_type const vXj = cross(v, j);
                
                vector3_type const wXi = cross(w, i);
                vector3_type const wXj = cross(w, j);
                
                J(3,0) = uXi(0);
                J(4,0) = uXi(1);
                J(5,0) = uXi(2);
                
                J(6,0) = uXj(0);
                J(7,0) = uXj(1);
                J(8,0) = uXj(2);
                
                J(3,1) = vXi(0);
                J(4,1) = vXi(1);
                J(5,1) = vXi(2);
                
                J(6,1) = vXj(0);
                J(7,1) = vXj(1);
                J(8,1) = vXj(2);
                
                J(3,2) = wXi(0);
                J(4,2) = wXi(1);
                J(5,2) = wXi(2);
                
                J(6,2) = wXj(0);
                J(7,2) = wXj(1);
                J(8,2) = wXj(2);
              }
            }
            else
            {
              assert(false || !"compute_jacobian(): unknown joint type");
            }
          }
          
        };
      
    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BONE_TRAITS_H
#endif
