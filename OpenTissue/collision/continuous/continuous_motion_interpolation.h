#ifndef OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_MOTION_INTERPOLATION_H
#define OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_MOTION_INTERPOLATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/continuous/continuous_conservative_advancement.h>

namespace OpenTissue
{
  namespace collision
  {
    namespace continuous
    {

      /**
      * Motion Interpolation.
      * This function tries to determine whether two objects have impacted
      * during their motion. The motion of the objects is specified by giving
      * initial and final positions of the objects. From these discrete poses
      * the function will reconstruct the continuous inbetween motion under the
      * assumption that the objects moved at contact linear and angular velocities
      * between their starting and finial positions.
      *
      * @param A_from          The initial placement of object A.
      * @param A_to            The final placement of object A.
      * @param A               The shape/geometry of object A.
      * @param r_max_a         Maximum radius of the shape of object A.
      * @param B_from          The initial placement of object B.
      * @param B_to            The final placement of object B.
      * @param B               The shape/geometry of object B.
      * @param r_max_b         Maximum radius of the shape of object B.
      * @param p_a             Upon return this argument holds the cloest point on object A in case of an impact.
      * @param p_b             Upon return this argument holds the cloest point on object B in case of an impact.
      * @param time_of_impact  Upon return if an impact is found then this argument holds the estimated value of the time of impact.
      * @param iterations      Upon return this argument holds the number of used iterations by the function. If the value is equal to the max_iterations argument then the function did not converge to an answer.
      * @param epsilon         The size of the collision envelope. That is the smallest separation distance between A and B where we consider A and B to be in touching contact.
      * @param max_iterations  The maximum number of allowed iterations that the function can take.
      * @param policy          A motion policy that provides the details of any sub-algorithms/routines needed by the function.
      *
      * @return                If an impact is found then the return value is true otherwise it is false.
      */
      template<typename transform_type, typename object_type1, typename object_type2, typename motion_policy>
      inline bool motion_interpolation(
        transform_type const & A_from
        , transform_type const & A_to
        , object_type1 & A
        , typename transform_type::value_type const & r_max_a
        , transform_type const & B_from
        , transform_type const & B_to
        , object_type2 & B
        , typename transform_type::value_type const & r_max_b
        , typename transform_type::vector3_type & p_a
        , typename transform_type::vector3_type & p_b
        , typename transform_type::value_type & time_of_impact
        , size_t & iterations
        , typename transform_type::value_type const & epsilon
        , size_t const & max_iterations 
        , motion_policy const & policy
        )
      {            
        typedef typename transform_type::value_traits    value_traits;
        typedef typename transform_type::vector3_type    V;
      
        V v_a;
        V omega_a;
        V v_b;
        V omega_b;

        motion_policy::compute_velocities( A_from, A_to, value_traits::one(), v_a, omega_a );
        motion_policy::compute_velocities( B_from, B_to, value_traits::one(), v_b, omega_b );

        return conservative_advancement(
          A_from      , v_a      , omega_a      , A      , r_max_a
          , B_from      , v_b      , omega_b      , B      , r_max_b
          , p_a
          , p_b
          , time_of_impact
          , iterations , epsilon, value_traits::one(), max_iterations, policy 
          );
      }

    } // namespace continuous
  } // namespace collision
} // namespace OpenTissue

// OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_MOTION_INTERPOLATION_H
#endif
