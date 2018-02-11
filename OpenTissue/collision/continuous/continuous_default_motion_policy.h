#ifndef OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_DEFAULT_MOTION_POLICY_H
#define OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_DEFAULT_MOTION_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>  
#include <OpenTissue/core/math/math_functions.h>  // Needed for clamp
#include <OpenTissue/collision/gjk/gjk_compute_closest_points.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace collision
  {
    namespace continuous
    {

      /**
      * Default Motion Policy.
      * Continuous collision detection algorithms often need to
      * interpretate continuous motion from discrete motion. That requires
      * converting discrete poses into velocities or interpolating motion etc..
      * Or simply to use a discrete collision detection system as a sub-system
      * in the continuous collision detection algorithm.
      *
      * This policy class contains default behaviour for such tasks.
      */
      class DefaultMotionPolicy
      {
      public:

        /**
        * Compute Velocities.
        * This function tries to convert two poses from the motion of an object into equivalent velocities.
        *
        * The function works under the assumption that the object in questions moves at constant
        * linear and angular velocities between the two given poses. The duration of the motion
        * is assumed to be one unit (second).
        *
        * @param T_from    A coordinate transformation indicating the starting pose of the motion.
        * @param T_to      A coordinate transformation indicating the ending pose of the motion.
        * @param delta_tau The time between from pose and to pose.
        * @param v         Upon return this argument holds the value of the constant linear velocity.
        * @param omega     Upon return this argument holds the value of the constant angular velocity.
        */
        template< typename transform_type >
        static void compute_velocities(
          transform_type const & T_from
          , transform_type const & T_to
          , typename transform_type::value_type const & delta_tau
          , typename transform_type::vector3_type & v
          , typename transform_type::vector3_type & omega
          )
        {
          using std::atan2;

          typedef typename transform_type::value_traits    value_traits;
          typedef typename transform_type::vector3_type    V;
          typedef typename transform_type::value_type      T;

          assert(  delta_tau > value_traits::zero() || !"compute_velocities(): time step must be positive");

          // Translation is straightforward
          v = (T_to.T() - T_from.T()) / delta_tau;

          T theta;
          V n;
          get_axis_angle(
              prod( T_to.Q(), conj( T_from.Q() ) )   // Change in orientation from ``from'' to ''to'', ie. R = T_to * T_from^{-1}
            , n
            , theta 
            );
          omega = (theta/ delta_tau)*n;
        }

        /**
        * Integrate Motion.
        *
        * @param X      The current coordinate transformation of the object.
        * @param tau    The time step into the future where the coordinate transformation of the object should be computed.
        * @param v      The current linear velocity of the object.
        * @param omega  The current angular velocity of the object. 
        *
        * @return       Upon return this argument holds the coordinate transformation of the object at time tau.
        */
        template< typename transform_type>
        static transform_type integrate_motion(
          transform_type const & X
          , typename transform_type::value_type const & tau
          , typename transform_type::vector3_type const & v
          , typename transform_type::vector3_type const & omega)
        {
          typedef typename transform_type::value_traits    value_traits;
          typedef typename transform_type::vector3_type    V;
          typedef typename transform_type::quaternion_type Q;
          typedef typename transform_type::value_type      T;

          assert( tau >= value_traits::zero() || !"integrate_motion(): Tau must be non-negative");

          T const radian           = tau * length( omega );
          V const axis             = unit( omega );

          assert( is_number( radian )  || !"integrate_motion(): NaN encountered");
          assert( is_number( axis(0) ) || !"integrate_motion(): NaN encountered");
          assert( is_number( axis(1) ) || !"integrate_motion(): NaN encountered");
          assert( is_number( axis(2) ) || !"integrate_motion(): NaN encountered");

          Q dq;
          V dv;
          dq.Ru( radian, axis);
          dv = v*tau;

          assert( is_number( dv(0) ) || !"integrate_motion(): NaN encountered");
          assert( is_number( dv(1) ) || !"integrate_motion(): NaN encountered");
          assert( is_number( dv(2) ) || !"integrate_motion(): NaN encountered");

          return transform_type( dv + X.T(), prod(dq, X.Q()) );
        }

        /**
        * Compute Closest Points.
        *
        * @param X_a   The current coordinate transformation of object A.
        * @param A     A support functor desribing the shape of object A.
        * @param X_b   The current coordinate transformation of object B.
        * @param B     A support functor desribing the shape of object B.
        * @param p_a   Upon return this argument holds the closest point on object A.
        * @param p_b   Upon return this argument holds the closest point on object B.
        */
        template< typename transform_type, typename object_type1, typename object_type2>
        static void compute_closest_points(
            transform_type const & X_a
          , object_type1 const & A
          , transform_type const & X_b
          , object_type2 const & B
          , typename transform_type::vector3_type & p_a
          , typename transform_type::vector3_type & p_b
          )
        {
          typedef typename transform_type::value_traits    value_traits;
          typedef typename transform_type::value_type      T;

          // Here we simply invoke our favorite collision detection sub-routine...
          //
          OpenTissue::gjk::VoronoiSimplexSolverPolicy const simplex_solver_policy = OpenTissue::gjk::VoronoiSimplexSolverPolicy();

          size_t const max_iterations       = 100u;
          T      const absolute_tolerance   = boost::numeric_cast<T>(10e-6);
          T      const relative_tolerance   = boost::numeric_cast<T>(10e-6);
          T      const stagnation_tolerance = boost::numeric_cast<T>(10e-15);
          size_t       iterations           = 0u;
          size_t       status               = 0u;
          T            distance             = value_traits::infinity();

          OpenTissue::gjk::compute_closest_points(
            X_a
            , A
            , X_b
            , B
            , p_a
            , p_b
            , distance
            , iterations
            , status
            , absolute_tolerance
            , relative_tolerance
            , stagnation_tolerance
            , max_iterations
            , simplex_solver_policy
            );

        }

      };

    } // namespace continuous
  } // namespace collision
} // namespace OpenTissue

// OPENTISSUE_COLLISION_CONTINUOUS_CONTINUOUS_DEFAULT_MOTION_POLICY_H
#endif
