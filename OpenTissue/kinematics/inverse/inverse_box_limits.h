#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BOX_LIMITS_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BOX_LIMITS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <cassert>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
      namespace box_limits
      {
        
        /**
         * Extract joint limits of a Bone.
         *
         * @param bone             The bone that should be worked on.
         * @param sub_min          A vector or vector range that upon return holds the minimum joint
         *                         limits of the specified bone.
         * @param sub_max          A vector or vector range that upon return holds the maximum joint
         *                         limits of the specified bone.
         */
        template<typename bone_type, typename vector_range>
        inline void get_joint_limits(bone_type const & bone, vector_range & sub_min,vector_range & sub_max)
        {
          assert(sub_min.size() == bone.active_dofs()         || !"get_joint_limits() invalid dimension");
          assert(sub_max.size() == bone.active_dofs()         || !"get_joint_limits() invalid dimension");
          
          if(bone.type() == bone_type::hinge_type)
          {
            sub_min(0) = bone.box_limits().min_limit(0);
            sub_max(0) = bone.box_limits().max_limit(0);
          }
          else if(bone.type() == bone_type::slider_type)
          {
            sub_min(0) = bone.box_limits().min_limit(0);
            sub_max(0) = bone.box_limits().max_limit(0);
          }
          else if(bone.type() == bone_type::ball_type)
          {
            sub_min(0) = bone.box_limits().min_limit(0);
            sub_min(1) = bone.box_limits().min_limit(1);
            sub_min(2) = bone.box_limits().min_limit(2);
            
            sub_max(0) = bone.box_limits().max_limit(0);
            sub_max(1) = bone.box_limits().max_limit(1);
            sub_max(2) = bone.box_limits().max_limit(2);
          }
          else
          {
            assert(false || !"get_joint_limits(): unknown joint type");
          }
        }
        
        
        /**
         * Project sub-vector onto joint limits of a Bone.
         *
         * @see BoneTraits::joint_limit_projection() for more details.
         */
        template<typename bone_type, typename vector_range>
        inline void joint_limit_projection( bone_type const & bone, vector_range & sub_theta)
        {
          using std::max;
          using std::min;
          
          assert(sub_theta.size() == bone.active_dofs() || !"joint_limit_projection() invalid dimension");
          assert(sub_theta.size() == bone.active_dofs() || !"joint_limit_projection() invalid dimension");
          
          if(bone.type() == bone_type::hinge_type || bone.type() == bone_type::slider_type)
          {
            sub_theta(0) = max( bone.box_limits().min_limit(0), min(bone.box_limits().max_limit(0), sub_theta(0) ) );
          }
          else if(bone.type() == bone_type::ball_type)
          {
            sub_theta(0) = max( bone.box_limits().min_limit(0), min(bone.box_limits().max_limit(0), sub_theta(0)) );
            sub_theta(1) = max( bone.box_limits().min_limit(1), min(bone.box_limits().max_limit(1), sub_theta(1)) );
            sub_theta(2) = max( bone.box_limits().min_limit(2), min(bone.box_limits().max_limit(2), sub_theta(2)) );
          }
          else
          {
            assert(false || !"joint_limit_projection(): unknown joint type");
          }
        }
        
        /**
         * Get upper and lower limits of the joint parameters.
         * This function will iterate over all the bones in the specified
         * skeleton and extract the joint limits for each bone and copy
         * these into agglomerated vectors.
         *
         * @param skeleton   The skeleton from which joint limits should be extracted.
         * @param min_theta  Upon return this argument holds the agglomerated vector of minimum joint limits.
         * @param max_theta  Upon return this argument holds the agglomerated vector of maximum joint limits.
         */
        template< typename skeleton_type>
        inline void get_joint_limits( 
                                     skeleton_type const & skeleton
                                     , ublas::vector<typename skeleton_type::bone_traits::real_type> & min_theta
                                     , ublas::vector<typename skeleton_type::bone_traits::real_type> & max_theta                
                                     )
        {
          using ublas::subrange;
          
          typedef typename skeleton_type::bone_traits              bone_traits;
          typedef typename bone_traits::real_type                  real_type;
          typedef          ublas::vector<real_type>                vector_type;
          typedef          ublas::vector_range<vector_type>        vector_range;
          
          // Determine size of theta
          size_t dofs = 0u;
          
          typename skeleton_type::const_bone_iterator bone = skeleton.begin();
          typename skeleton_type::const_bone_iterator end  = skeleton.end();
          
          for(;bone!=end;++bone)
          {
            dofs += bone->active_dofs(); 
          }
          
          //make sure theta has proper size
          if(min_theta.size() != dofs)
            min_theta.resize( dofs );
          if(max_theta.size() != dofs)
            max_theta.resize( dofs );
          
          //Now retrieve joint parameter values from each bone and store it into theta
          size_t i   = 0u;
          size_t dof = 0u;
          
          bone = skeleton.begin();
          for(;bone!=end;++bone)
          {
            dof = bone->active_dofs(); 
            
            vector_range sub_min = subrange(min_theta,i,i+dof);
            vector_range sub_max = subrange(max_theta,i,i+dof);
            OpenTissue::kinematics::inverse::box_limits::get_joint_limits( *bone , sub_min, sub_max);
            
            i += dof;
          }
        }
        
      } // namespace box_limits
      
      /**
       * Box Limits Class.
       * This class holds the data necessary for representing box-limits. The class is no-more than a
       * pure data-class. All algorithms and functionality can be found in the box_limits namespace.
       */
      template<typename bone_traits >
      class BoxLimits 
        {
        public:
          
          typedef typename bone_traits::math_types::real_type        real_type;
          typedef typename bone_traits::math_types::value_traits     value_traits;
          
        protected:
          
          real_type             m_min_theta[6];      ///< Minimum joint parameter value.
          real_type             m_max_theta[6];      ///< Maximum joint parameter value.
          
        public:
          
          BoxLimits()
          {
            for(size_t i=0;i<6;++i)
            {
              m_min_theta[i] = -value_traits::pi();
              m_max_theta[i] =  value_traits::pi();    
            }
          }
          
          /**
           * Get Minimum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type const & min_limit(size_t const & i) const 
          {
            assert( i <  6u || !"min_joint_limit(): invalid index value");
            return m_min_theta[i];
          }
          
          /**
           * Get Minimum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type      & min_limit(size_t const & i)
          {
            assert( i < 6u || !"min_joint_limit(): invalid index value");
            return m_min_theta[i];
          }
          
          /**
           * Get Maximum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type const & max_limit(size_t const & i) const 
          {
            assert( i < 6u || !"max_joint_limit(): invalid index value");
            return m_max_theta[i];
          }
          
          /**
           * Get Maximum Joint Limit.
           *
           * @param i   The index of the value for which the limits is wanted.
           * @return    A reference to the corresponding joint limit.  
           */
          real_type      & max_limit(size_t const & i)
          {
            assert( i < 6u || !"max_limit(): invalid index value");
            return m_max_theta[i];
          }
          
        };
      
    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_BOX_LIMITS_H
#endif
