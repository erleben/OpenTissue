#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_SET_JOINT_PARAMETERS_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_SET_JOINT_PARAMETERS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <OpenTissue/kinematics/inverse/inverse_accessor.h>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Set Joint Parameter Values. 
      * This function takes an agglomerated vector of joint parameters and transfer
      * its values onto the bones/joints of a skeleton. During the process the relative
      * transform of the bones in the skeleton are updated and upon completion before
      * returning the absolute transformations of the bones are computed.
      *
      * @param skeleton  The skeleton where joint parameters should be set.
      * @param theta     Agglomerated vector of joint parameter values.
      */
      template<typename skeleton_type>
      void set_joint_parameters(
          skeleton_type & skeleton
        , ublas::vector<typename skeleton_type::bone_traits::real_type> const & theta 
        )
      {
        using ublas::subrange;

        typedef typename skeleton_type::bone_traits              bone_traits;
        typedef typename bone_traits::real_type                  real_type;
        typedef          ublas::vector<real_type>                vector_type;
        typedef          ublas::vector_range<const vector_type>  const_vector_range;

        size_t i   = 0u;
        size_t dof = 0u;

        typename skeleton_type::bone_iterator bone = skeleton.begin();
        typename skeleton_type::bone_iterator end  = skeleton.end();
        for(;bone!=end;++bone)
        {
          dof = bone->active_dofs();
          // Get the sub-block of theta corresponding to the current bone
          const_vector_range sub_theta = subrange(theta,i,(i+dof));
          // Set the relative bone transform of each bone
          ACCESSOR::set_theta( *bone, sub_theta);
          i += dof;
        }
        // Finally compute absolute bone transformations
        skeleton.compute_pose();
      }
      
      /**
       * Set  Joint Paramter Values.
       * This convenience function tries to extract meningful joint parameter
       * settings from a given skeleton from the current relative bone
       * transformations and joint types.
       *
       * The function is intended to aid end-users in faster and easier rigging
       * of characters. The idea is that the function tries to come up with some
       * good guess for the joint parameter values and the joint limits. Hereafter an
       * end-user should fine tune and adjust the settings.
       *
       * @warning Make sure that the relative bone transforms have been set prior to invoking
       * this method. This could for instance be done by
       * invoking the skelton.set_bind_pose() method. Also make sure to set the
       * joint types of each bone. If not the default joint type will be assumed.
       *
       * @param skeleton        The skeleton that the function should work on.
       * @param use_bind_pose   If set to true (the default value) then the joint parameters
       *                        will be set from the value of the bind pose. If set to false
       *                        the joint parameters will be set from the curent value of the
       *                        relative bone transformation.
       *
       */
      template<typename skeleton_type>
      inline void  set_joint_parameters( skeleton_type & skeleton, bool const & use_bind_pose = true )
      {
        typedef typename skeleton_type::bone_type             bone_type;
        typedef typename skeleton_type::bone_iterator         bone_iterator;
        
        bone_iterator    bone = skeleton.begin();  
        bone_iterator    end  = skeleton.end();
        for( ; bone!=end; ++bone)
          ACCESSOR::set_theta_from_bone_transformation( *bone, use_bind_pose );        
      }
      
    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_SET_JOINT_PARAMETERS_H
#endif
