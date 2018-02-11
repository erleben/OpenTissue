#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_WEIGHTED_DIFFERENCE_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_WEIGHTED_DIFFERENCE_H
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

      /**
      * Compute Weighted Goal End-effector Difference Vector.
      * This function computes the vector given by
      *
      * \f[ W (g-F(\Theta)) \f]
      *
      *
      * @param begin      An iterator to the first chain.
      * @param end        An iterator to one past the last chain.
      * @param delta      upon return this holds the weighted difference between all end 
      *                   effectors and all goals
      */
      template<typename chain_iterator, typename T>
      inline void compute_weighted_difference( 
          chain_iterator const & begin
        , chain_iterator const & end
        , ublas::vector<T> &  delta 
        )
      {
        using ublas::subrange;

        typedef typename chain_iterator::value_type        chain_type;
        typedef typename chain_type::bone_traits           bone_traits;
        typedef typename chain_type::vector3_type          vector3_type;
        typedef          ublas::vector<T>                  vector_type;
        typedef          ublas::vector_range<vector_type>  vector_range;

        size_t index = 0u;
        size_t N     = 0u;

        chain_iterator chain = begin;

        for(; chain!=end; ++chain)
          N +=  chain->get_goal_dimension();

        if(delta.size() != N)
          delta.resize(N);

        for(chain = begin; chain!=end; ++chain)
        { 
          assert( chain->get_goal_dimension()==3 || chain->get_goal_dimension()==9 || !"compute_weighted_difference(): only goals of dimension 3 or 9 are currently supported");

          vector_range sub_delta = subrange(delta,index,(index+chain->get_goal_dimension()));

          vector3_type delta_p =  chain->p_global() -  bone_traits::transform_point( chain->get_end_effector()->absolute(), chain->p_local() );

          sub_delta(0) = chain->weight_p() * delta_p[0];
          sub_delta(1) = chain->weight_p() * delta_p[1];
          sub_delta(2) = chain->weight_p() * delta_p[2];

          if( chain->get_goal_dimension() == 9u )
          {
            vector3_type delta_x = chain->x_global() - bone_traits::transform_vector( chain->get_end_effector()->absolute(), chain->x_local() );
            vector3_type delta_y = chain->y_global() - bone_traits::transform_vector( chain->get_end_effector()->absolute(), chain->y_local() );

            sub_delta(3) = chain->weight_x() * delta_x(0);
            sub_delta(4) = chain->weight_x() * delta_x(1);
            sub_delta(5) = chain->weight_x() * delta_x(2);

            sub_delta(6) = chain->weight_y() * delta_y(0);
            sub_delta(7) = chain->weight_y() * delta_y(1);
            sub_delta(8) = chain->weight_y() * delta_y(2);
          }

          index += chain->get_goal_dimension();
        }
      }

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_WEIGHTED_DIFFERENCE_H
#endif
