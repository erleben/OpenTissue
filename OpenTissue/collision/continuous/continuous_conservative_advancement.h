#ifndef OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_CONSERVATIVE_ADVANCEMENT_H
#define OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_CONSERVATIVE_ADVANCEMENT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace collision
  {
    namespace continuous
    {


      /**
      * Conservative Advancement.
      * This function tries to determine whether two objects have impacted
      * during their motion. The function assumes that the objects will move
      * continuously in the future with contact linear and angular velocities.
      *
      * @param T_a             The initial placement of object A.
      * @param v_a             The contact linear velocity of object A.
      * @param w_a             The contact angular velocity of object A.
      * @param A               The shape/geometry of object A.
      * @param r_max_a         Maximum radius of the shape of object A.
      * @param T_b             The initial placement of object B.
      * @param v_b             The contact linear velocity of object B.
      * @param w_b             The contact angular velocity of object B.
      * @param B               The shape/geometry of object B.
      * @param r_max_b         Maximum radius of the shape of object B.
      * @param p_a             Upon return this argument holds the cloest point on object A in case of an impact.
      * @param p_b             Upon return this argument holds the cloest point on object B in case of an impact.
      * @param time_of_impact  Upon return if an impact is found then this argument holds the estimated value of the time of impact.
      * @param iterations      Upon return this argument holds the number of used iterations by the function. If the value is equal to the max_iterations argument then the function did not converge to an answer.
      * @param epsilon         The size of the collision envelope. That is the smallest separation distance between A and B where we consider A and B to be in touching contact.
      * @param max_tau         The maximum time into the future that the function will look for a time of impact.
      * @param max_iterations  The maximum number of allowed iterations that the function can take.
      * @param policy          A motion policy that provides the details of any sub-algorithms/routines needed by the function.
      *
      * @return                If an impact is found then the return value is true otherwise it is false.
      */
      template<typename transform_type, typename object_type1, typename object_type2, typename motion_policy>
      inline bool conservative_advancement(
          transform_type const & T_a
        , typename transform_type::vector3_type const & v_a
        , typename transform_type::vector3_type const & omega_a
        , object_type1 & A
        , typename transform_type::value_type const & r_max_a
        , transform_type const & T_b
        , typename transform_type::vector3_type const & v_b
        , typename transform_type::vector3_type const & omega_b
        , object_type2 & B
        , typename transform_type::value_type const & r_max_b
        , typename transform_type::vector3_type & p_a
        , typename transform_type::vector3_type & p_b
        , typename transform_type::value_type & time_of_impact
        , size_t & iterations
        , typename transform_type::value_type const & epsilon
        , typename transform_type::value_type const & max_tau
        , size_t const & max_iterations 
        , motion_policy const & /*policy*/
        )
      {            
        typedef typename transform_type::value_traits    value_traits;
        typedef typename transform_type::vector3_type    V;
        typedef typename transform_type::value_type      T;

        assert( r_max_a > value_traits::zero() || !"conservative_advancement(): maximum distance of object A must be positive");
        assert( r_max_b > value_traits::zero() || !"conservative_advancement(): maximum distance of object B must be positive");
        assert( max_tau > value_traits::zero() || !"conservative_advancement(): maximum time-step must be positive");
        assert( epsilon > value_traits::zero() || !"conservative_advancement(): collision envelope must be positive");
        assert( max_iterations > 0u            || !"conservative_advancement(): maximum iterations must be positive");

        T tau = value_traits::zero();

        for(iterations=1u; iterations <= max_iterations; ++iterations)
        {
          // Compute the coordinate transformations corresponding to the current tau value
          transform_type X_a = motion_policy::integrate_motion( T_a, tau, v_a, omega_a );
          transform_type X_b = motion_policy::integrate_motion( T_b, tau, v_b, omega_b );

          // Compute the closest points at the time tau
          motion_policy::compute_closest_points( X_a, A, X_b, B, p_a, p_b );

          // Estimate normal direction and current minimum distance between A and B
          V v = p_a - p_b;

          T min_distance = length(v);

          if( min_distance <= epsilon )
          {
            time_of_impact = tau;
            return true;
          }

          V n = unit( v );

          // Estimate maximum relative normal velocity between any two points from A and B

          T max_velocity = dot(v_b - v_a, n) + length(omega_a)*r_max_a + length(omega_b)*r_max_b;

          if (max_velocity <= value_traits::zero() )
            return false;

          // Compute conservative lower bound for when A and B could impact
          T delta_tau = min_distance / max_velocity;
          tau += delta_tau;

          if ( tau > max_tau ) 
            return false;
        }

        // not enough iterations to determine what goes on! We give up
        return false;
      }


    } // namespace continuous
  } // namespace collision
} // namespace OpenTissue

// OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_CONSERVATIVE_ADVANCEMENT_H
#endif
