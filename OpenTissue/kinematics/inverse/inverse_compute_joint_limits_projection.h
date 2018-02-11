#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_JOINT_LIMITS_PROJECTION_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_JOINT_LIMITS_PROJECTION_H
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

#include <cassert>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Compute projection onto upper and lower joint limits.
      * This function will iterate over all the bones in the specified
      * skeleton and extract the joint limits for each bone and project
      * the corresponding sub-block of the agglomerated joint parameter
      * vector onto these limits.
      *
      *
      * @param skeleton  The skeleton from which joint parameter values
      *                  should be extracted.
      * @param theta     Upon invokation this argument holds the current
      *                  value of the agglomerated joint parameter values
      *                  and Upon return it  holds the projected values.
      *
      */
      template< typename skeleton_type>
      inline void compute_joint_limits_projection( 
          skeleton_type const & skeleton
        , ublas::vector<typename skeleton_type::bone_traits::real_type> & theta
        )
      {
        using ublas::subrange;

        typedef typename skeleton_type::bone_traits              bone_traits;
        typedef typename bone_traits::real_type                  real_type;
        typedef          ublas::vector<real_type>                vector_type;
        typedef          ublas::vector_range<vector_type>        vector_range;

        typename skeleton_type::const_bone_iterator bone = skeleton.begin();
        typename skeleton_type::const_bone_iterator end  = skeleton.end();

        size_t i    = 0u;
        size_t dof  = 0u;

        for(;bone!=end;++bone)
        {
          dof = bone->active_dofs(); 

          vector_range sub_theta = subrange(theta,i,i+dof);

          ACCESSOR::joint_limit_projection( *bone , sub_theta );

          i += dof;
        }

        assert( theta.size() == i || !"compute_joint_limits_projection(): incompatible dimensions");
      }

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_JOINT_LIMITS_PROJECTION_H
#endif
