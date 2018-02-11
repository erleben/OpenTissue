#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_GET_JOINT_PARAMETERS_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_GET_JOINT_PARAMETERS_H
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
      * Get Joint Parameter Values.
      * This function will iterate over all the bones in the specified
      * skeleton and extract the joint parameters for each bone and copy
      * these into an agglomerated vector.
      *
      * @param skeleton        The skeleton from which joint parameter values should be extracted.
      * @param theta           Upon return this argument holds the agglomerated vector of joint parameter values.
      * @param resynchronize   @see ACCESSOR::get_theta() for description.   
      */
      template< typename skeleton_type>
      inline void get_joint_parameters( 
          skeleton_type & skeleton
        , ublas::vector<typename skeleton_type::bone_traits::real_type> & theta
        , bool const & resynchronize = false                                
        )
      {
        using ublas::subrange;

        typedef typename skeleton_type::bone_traits              bone_traits;
        typedef typename bone_traits::real_type                  real_type;
        typedef          ublas::vector<real_type>                vector_type;
        typedef          ublas::vector_range<vector_type>        vector_range;

        // Determine size of theta
        size_t dofs = 0u;

        typename skeleton_type::bone_iterator bone = skeleton.begin();
        typename skeleton_type::bone_iterator end  = skeleton.end();

        for(;bone!=end;++bone)
          dofs += bone->active_dofs(); 

        //make sure theta has proper size
        if(theta.size() != dofs )
          theta.resize( dofs );

        //Now retrive joint parameter values from each bone and store it into theta
        size_t i   = 0u;
        size_t dof = 0u;

        bone = skeleton.begin();
        for(;bone!=end;++bone)
        {
          dof = bone->active_dofs(); 

          vector_range sub_theta = subrange(theta,i,i+dof);        

          ACCESSOR::get_theta( *bone , sub_theta, resynchronize);

          i += dof;
        }
      }


    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_GET_JOINT_PARAMETERS_H
#endif
