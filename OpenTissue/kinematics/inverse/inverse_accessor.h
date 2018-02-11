#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_ACCESSOR_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_ACCESSOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_euler_angles.h> // needed for ZYZ_euler_angles

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
            
      /**
       * The ACCESSOR class.
       * This class provides a back-door for all IK algorithms to directly access and
       * manipulate IK related data stored in bones.
       *
       * The intention of the ACCESSOR class is to hide this direct access to data from
       * the common user. It could be very dangerous to use the methods of the ACCESSOR
       * class if one does not have knowledge of any side-effects.
       */
      class ACCESSOR
        {
        public:

          /**
           * Unsynchronized Set Theta Value.
           * This method chnages the i'th joint parameter value of
           * the specified bone. The method access the joint parameter value in a raw and
           * unsynchronized manner. That is any coordinate transformation data stored in
           * the base class of the bone is completely ignored. In other words if a theta
           * value is changed using this paper then the corresponding relative bone transformations
           * are no longer in synch with the theta-values.
           *
           * The method is mostly intended for internal usage by advanced methods or algorithms where
           * one have full control over any synchronization between bone transformations and inverse
           * kinematics parameter values.
           *
           * @param bone    A reference to the bone.
           * @param i       The index of the joint parameter value.
           * @return        The value of the i'th joint parameter value stored in the specified bone.
           */
          template<typename bone_type>
          static void unsynch_set_theta( bone_type & bone,  size_t const & i, typename bone_type::math_types::real_type const & value)
          {
            bone.theta(i) = value;
          }
          
          /**
           * Unsynchronized Get Theta Value.
           * This method returns a const reference to the i'th joint parameter value of
           * the specified bone. The method access the joint parameter value in a raw and
           * unsynchronized manner. That is any coordinate transformation data stored in
           * the base class of the bone is completely ignored.
           *
           * The method is mostly intended for debugging/testing purpose where one wants
           * direct access to the data values stored in the inverse kinematics bone trait class.
           *
           * @param bone    A reference to the bone.
           * @param i       The index of the joint parameter value.
           * @return        The value of the i'th joint parameter value stored in the specified bone.
           */
          template<typename bone_type>
          static typename bone_type::math_types::real_type const &  unsynch_get_theta( bone_type const & bone,  size_t const & i)
          {
            return bone.theta(i);
          }
    
          /**
           * Set Joint Parameter Values.
           * This method reads the joint parameter values from a specified
           * sub-vector. The function copies the values into the local storage
           * in this bone and further it sets the relative transformation of the
           * bone to match the joint parameter values.
           *
           * It is rather important that the relative transformation of the bone
           * is updated accordingly otherwise other libraries like skinning and
           * animation will stop working correctly when used together with
           * the inverse kinematics library.
           *
           * @warning  Observe that after having set the joint parameter values of all
           *           bones then one should also re-compute the absolute bone transformations
           *           if these are needed. The inverse kinematics library will do this by
           *           default.
           *
           *
           * @param bone             The bone that should be worked on. 
           * @param sub_theta        A vector or vector range from which the joint
           *                         parameters should be extracted.
           */
          template<typename bone_type, typename vector_range>
          static void set_theta(bone_type & bone, vector_range const & sub_theta)
          {
            typedef typename bone_type::math_types::real_type  real_type;
            
            assert(sub_theta.size() == bone.active_dofs() || !"set_theta() invalid dimension");
            
            if(bone.type() == bone_type::hinge_type)
            {
              bone.theta(0) = sub_theta(0);
              bone.relative().Q() = OpenTissue::math::Ru(bone.theta(0), bone.u() );
            }
            else if(bone.type() == bone_type::slider_type)
            {
              bone.theta(0) = sub_theta(0);
              bone.relative().T() = bone.theta(0) * bone.u();
            }
            else if(bone.type() == bone_type::ball_type)
            {
              real_type phi   = (bone.theta(0) = sub_theta(0) );
              real_type psi   = (bone.theta(1) = sub_theta(1) );
              real_type theta = (bone.theta(2) = sub_theta(2) );
              
              bone.relative().Q() = OpenTissue::math::Rz( phi )*OpenTissue::math::Ry( psi )*OpenTissue::math::Rz(theta);
            }
            else
            {
              assert(false || !"set_theta(): unknown joint type");
            }
          }
          
          /**
           * Extract Joint Parameter Values of a Bone.
           * This mehtod implicitly assumes that the inverse kinematics bone parameters are
           * in synch with the bone transformations. Thus if one changes the relative bone
           * transformations and then afterwards wants to extract the joint parameter values
           * then the extracted values would correspond to the old setting of the relative
           * bone transformation. Thus one should use this method with great care.
           *
           * @param bone             The bone that should be worked on.
           * @param sub_theta        A vector or vector range that upon return holds the joint
           *                         parameter values of the specified bone.
           * @param resynchronize   In case an end-user has directly manipulated the relative
           *                        bone coordinate transformations then the IK data will have become
           *                        out of synch with the relative bone coordinate transformations. To
           *                        fix the problem one must resynchronize the IK data with the relative
           *                        bone transformations. This can for instance be done by setting this
           *                        flag. In most cases an end-user will only perform IK and refrain from
           *                        direct manipulation (ie. forward kineamtics). Thus for performace reasons
           *                        one should turn this flag off unless one knows that synchronization between
           *                        directly manipulated data and IK data is called for. This is the default behaviour.   
           */
          template<typename bone_type, typename vector_range>
          static void get_theta(
                                  bone_type & bone
                                , vector_range & sub_theta
                                , bool const & resynchronize = false                                
                                )
          {
            assert(sub_theta.size() == bone.active_dofs()         || !"get_theta() invalid dimension");
            
            if( resynchronize )
              set_theta_from_bone_transformation( bone, false ); 
            
            if(bone.type() == bone_type::hinge_type)
            {
              sub_theta(0) = bone.theta(0);
            }
            else if(bone.type() == bone_type::slider_type)
            {
              sub_theta(0) = bone.theta(0);
            }
            else if(bone.type() == bone_type::ball_type)
            {
              sub_theta(0) = bone.theta(0);
              sub_theta(1) = bone.theta(1);
              sub_theta(2) = bone.theta(2);
            }
            else
            {
              assert(false || !"get_theta(): unknown joint type");
            }
          }
          
          /**
           * Synchronize Inverse Kinematics Joint Parameters from Bone transformations. 
           * This method makes sure the joint parameters, the theta values, of a bone is equivalent to the actual
           * pose of the bone. By default it uses the relative bone transformation of the bone. It is 
           * useful if one has performed forward kinematics on the skeleton (ie. manipulated bone.relative() directly) and
           * wants to set up the IK to match the current pose.
           *
           * @param  bone           The bone which should be synchronized.
           * @param use_bind_pose   If set to true then the joint parameters
           *                        will be set from the value of the bind pose. If set to false
           *                        the joint parameters will be set from the curent value of the
           *                        relative bone transformation.
           *
           */
          template<typename bone_type>
          static void set_theta_from_bone_transformation(bone_type & bone,  bool const & use_bind_pose = true)
          {
            using std::atan2;
            
            typedef typename bone_type::bone_traits             bone_traits;
            typedef typename bone_traits::transform_type        transform_type;
            typedef typename bone_traits::real_type             real_type;
            typedef typename bone_traits::value_traits          value_traits;
            typedef typename bone_traits::vector3_type          vector3_type;
            typedef typename transform_type::quaternion_type    quaternion_type;
            
            transform_type const & T = use_bind_pose ? bone.bind_pose() : bone.relative();
            
            if(bone.type() == bone_traits::hinge_type)
            {
              quaternion_type Q = T.Q();
              vector3_type u = unit( Q.v() );
              // we know
              //
              // Q = [cos(theta/2), sin(theta/2) u]
              //
              real_type const ct2    = Q.s();           //---   cos(theta/2)
              real_type const st2    = length( Q.v() ); //---  |sin(theta/2)|
              real_type const theta  = value_traits::two()* atan2(st2,ct2);
              
              // of course it might have been -|sin(theta/2)| that were the correct solution?
              bone.u()      = u;
              bone.theta(0) = theta;
            }
            else if(bone.type() == bone_traits::slider_type)
            {
              vector3_type const u     = unit( T.T() );
              real_type    const theta = length( T.T() );
              bone.u()      = u;
              bone.theta(0) = theta;
            }
            else if(bone.type() == bone_traits::ball_type)
            {
              real_type phi     = value_traits::zero();
              real_type psi     = value_traits::zero();
              real_type theta   = value_traits::zero();
              
              quaternion_type Q = T.Q();
              
              OpenTissue::math::ZYZ_euler_angles(Q, phi, psi, theta);
              
              // Now we can store the correct joint parameter values into the bone
              bone.theta(0) = phi;
              bone.theta(1) = psi;
              bone.theta(2) = theta;
            }
            else
            {
              assert(false || !"set_theta_from_bone_transformation(): unsupported joint type encountered!");
            }
          }
          
          /**
           * Project sub-vector onto joint limits of a Bone.
           *
           * @see BoneTraits::joint_limit_projection().
           *
           */
          template<typename bone_type, typename vector_range>
          static void joint_limit_projection( bone_type const & bone, vector_range & sub_theta)
          {
            bone_type::bone_traits::joint_limit_projection( bone, sub_theta );
          }
          
          /**
           * Computes the Jacobian matrix corresponding to this bone
           *
           * @see BoneTraits::compute_jacobian().
           */
          template<typename bone_type, typename chain_type, typename matrix_range>
          static void compute_jacobian( bone_type const & bone, chain_type const & chain, matrix_range & J )
          {
            bone_type::bone_traits::compute_jacobian( bone, chain, J );
          }
                    
        };
            
    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_ACCESSOR_H
#endif
