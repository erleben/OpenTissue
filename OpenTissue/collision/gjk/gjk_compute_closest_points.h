#ifndef OPENTISSUE_COLLISION_GJK_GJK_COMPUTE_CLOSEST_POINTS_H
#define OPENTISSUE_COLLISION_GJK_GJK_COMPUTE_CLOSEST_POINTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/gjk/gjk_constants.h>
#include <OpenTissue/collision/gjk/gjk_simplex.h>
#include <OpenTissue/collision/gjk/gjk_voronoi_simplex_solver_policy.h>

#include <cmath>
#include <stdexcept>
#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {

    /**
    * Compute Closest Points and Distance between two Convex Sets.
    * This function uses an iterative algorithm. In each iteration a simplex
    * is updated to best approximate the Minikowsky difference between two
    * given convex objects \f$A\f$ and \f$B\f$. The Minikowsky Difference set
    * is implicitly represented by the support functions of the two objects.
    * The following text tries to give a short introduction to the concepts.
    *
    * Given two objects represented by the two sets \f$A\f$ and \f$B\f$
    * then the two objects overlap if there exist at least one
    * \f$a \in A\f$ and one \f$b \in B\f$ such that
    * 
    *   \f[
    *     a - b = 0  
    *   \f]
    * 
    * Thus one way to test for intersection of the two objects is to form the set
    * 
    *   \f[
    *     A \ominus B = \{ a - b | a \in A, b \in B \}  
    *   \f]
    * 
    * and see if
    * 
    *   \f[
    *     0 \in A \ominus B  
    *   \f]
    * 
    * The set \f$A \ominus B\f$ is called the Minokowsky difference. Clearly if the two
    * objects are separated one must have that
    * 
    *   \f[
    *     a - b \neq 0  
    *   \f]
    * 
    * holds for all \f$a \in A\f$ and all \f$b \in B\f$, or equivalently that
    * 
    *   \f[
    *     0 \notin A \ominus B  
    *   \f]
    * 
    * Assume that the two objects are separated then one may want to know the minimum
    * distance between the two objects. That is one wants to find
    * 
    *   \f[
    *     (a^*, b^*) = \min_{a \in A, b \in B} \norm{(a-b)} = \min_{y \in A \ominus B} \norm{(y)}
    *   \f]
    * 
    * or equivalently
    * 
    *   \f[
    *     y^* = \min_{y \in A \ominus B} \norm{(y)}
    *   \f]
    * 
    * Thus seeking the minimum distance between the two object is equivalent to
    * finding the minimum norm point in the set \f$A \ominus B\f$. Or said differently to
    * find a point in \f$A \ominus B\f$ that is closest to zero.
    * 
    * Observe that if we find such a point \f$y^* \in A \ominus B\f$ then we implicitly
    * also know the two closest points between \f$A\f$ and \f$B\f$, since \f$y^*\f$ is defined as
    * \f$y^* = a^* - b^*\f$ for some \f$a^* \in A\f$ and some \f$b^* \in B\f$.
    * 
    * The important thing to realise is that the problem of finding the minimum
    * distance between two sets \f$A\f$ and \f$B\f$ is equivalent to the problem of finding
    * the distance between a point and the set \f$A \ominus B\f$. Thus we have replaced
    * the original problem with a simple one.
    * 
    * One should notice that the solution may not be unique since there could exist
    * multiple \f$y^* \in A \ominus B\f$ that yield the same minimum distance. Further a
    * solution may not exist if \f$0 \in A \ominus B\f$.
    *
    *
    *
    *
    *
    *
    *
    * @param transform_A          A coordinate (a rigid body) transformation that is used to place first convex set.
    * @param support_function_A   A support function describing the shape of the first convex set.
    * @param transform_B          A coordinate (a rigid body) transformation that is used to place second convex set.
    * @param support_function_B   A support function describing the shape of the second convex set.
    *
    * @param p_a                  Upon return this argument holds the closest point on the first convex set.
    * @param p_b                  Upon return this argument holds the closest point on the second convex set.
    *
    * @param distance             Upon return this argument holds the distance between the point and the convex set.
    *
    * @param iterations           Upon return this argument holds the number of iterations used.
    * @param status               The status code of the algorithm.
    * @param absolute_tolerance   
    * @param relative_tolerance   
    * @param stagnation_tolerance   
    * @param max_iterations
    *
    * @tparam transform_type           This template argument provides a coordinate transformation
    *                                  type. The type must support that one can query a quaternion
    *                                  representation of the orientation and a vector3_type representation
    *                                  of the translation. The OpenTissue CoordSys template class can be
    *                                  used for creating the type.
    * @tparam simplex_solver_policy    This template argument provides the algorithm with a policy for
    *                                  how to deal with the simplex approximation. 
    */
    template<
      typename transform_type
      , typename support_functor1
      , typename support_functor2
      , typename simplex_solver_policy
    >
    inline void compute_closest_points( 
      transform_type const & transform_A
    , support_functor1 const & support_function_A
    , transform_type const & transform_B
    , support_functor2 const & support_function_B
    , typename transform_type::vector3_type & p_a
    , typename transform_type::vector3_type & p_b
    , typename transform_type::value_type & distance
    , size_t & iterations
    , size_t & status
    , typename transform_type::value_type const & absolute_tolerance
    , typename transform_type::value_type const & relative_tolerance
    , typename transform_type::value_type const & stagnation_tolerance
    , size_t const & max_iterations
    , simplex_solver_policy const & /*solver_policy*/
    )
    {
      using std::sqrt;
      using std::fabs;
      using std::max;

      typedef typename transform_type::vector3_type  V;
      typedef typename transform_type::value_type    T;
      typedef typename transform_type::value_traits  value_traits;
      typedef          Simplex<V>                    simplex_type;

      if( absolute_tolerance < value_traits::zero() ) 
        throw std::invalid_argument( "absolute tolerance must be non-negative" );
      if( relative_tolerance < value_traits::zero() ) 
        throw std::invalid_argument( "relative tolerance must be non-negative" );
      if( stagnation_tolerance < value_traits::zero() ) 
        throw std::invalid_argument( "stagnation tolerance must be non-negative" );
      if( max_iterations <= 0u ) 
        throw std::invalid_argument( "max_iterations must be positive" );

      distance   = value_traits::infinity();
      status     = ITERATING;
      iterations = 0u;

      T    const squared_absolute_tolerance = absolute_tolerance*absolute_tolerance;
      T          squared_distance           = value_traits::infinity();

      // Simplex approximation to convex set C
      simplex_type sigma;

      // Initially we use a 0-simplex corresponding to some point
      // in C. We do this by seeding the initial closest point to
      // be the zero-vector.
      V v = V( value_traits::zero(), value_traits::zero(), value_traits::zero() );


      // Lower error bound on distance from origin to closest point 
      T mu = value_traits::zero();

      // We use a maximum iteration count to guard against infinite loops.
      for(iterations=1u; iterations<=max_iterations; ++iterations)
      {
        // Find another point w that is hopefully closer to the origin.
        //
        // That means the search direction should point towards the origin, ie. s = -v
        //
        //   S_{ T_A(A) - T_B(B) }(s) = S_{ T_A(A) } (-v) - S_{ T_B(B) }(v)
        //                            = T_A( S_A(- R_A^T v) ) - T_B( S_B(R_B^T v)
        //
        V s_a = conj( transform_A.Q() ).rotate( - v );
        V s_b = conj( transform_B.Q() ).rotate(   v );

        V w_a = support_function_A( s_a );
        V w_b = support_function_B( s_b );

        w_a = transform_A.Q().rotate( w_a ) + transform_A.T();
        w_b = transform_B.Q().rotate( w_b ) + transform_B.T();

        assert( is_number( w_a(0) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( w_a(1) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( w_a(2) ) || !"compute_closest_points(): NaN encountered");

        assert( is_number( w_b(0) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( w_b(1) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( w_b(2) ) || !"compute_closest_points(): NaN encountered");

        V w    = w_a - w_b;

        assert( is_number( w(0) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( w(1) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( w(2) ) || !"compute_closest_points(): NaN encountered");

        // Test if the new point is already part of the current simplex
        if ( is_point_in_simplex ( w, sigma ) )
        {
          assert( iterations > 1u || !"compute_closest_points(): simplex should be empty in first iteration?");
          // if so it means we can not find any points in C that is
          // closer to p and we are done
          distance = sqrt( squared_distance );
          status = SIMPLEX_EXPANSION_FAILED;
          return;
        }

        // Update lower error bound
        distance = sqrt(squared_distance);
        mu = max( mu, ( dot(v,w) / distance) );
        // Test relative stopping criteria proposed by Gino van den Bergen!
        if(distance - mu <= distance * relative_tolerance)
        {
          status = LOWER_ERROR_BOUND_CONVERGENCE;
          return;
        }

        // Extend the simplex with a new vertex
        add_point_to_simplex(w, w_a, w_b, sigma);

        // Compute the point, v, on the simplex that is closest to the origin and
        // Reduce simplex to lowest dimensional face on the boundary
        // containing the closest point.
        v = simplex_solver_policy::reduce_simplex( sigma, p_a, p_b );

        assert( is_number( v(0) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( v(1) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( v(2) ) || !"compute_closest_points(): NaN encountered");

        assert( is_number( p_a(0) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( p_a(1) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( p_a(2) ) || !"compute_closest_points(): NaN encountered");

        assert( is_number( p_b(0) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( p_b(1) ) || !"compute_closest_points(): NaN encountered");
        assert( is_number( p_b(2) ) || !"compute_closest_points(): NaN encountered");

        // Test if simplex is a full tetrahedron. In this case the closest
        // point must be inside the tetrahedron and equal to the origin. Thus
        // we clearly have an intersection.
        if( is_full_simplex( sigma) )
        {
          distance = value_traits::zero();
          status = INTERSECTION;
          return;
        }

        T const old_squared_distance = squared_distance;
        squared_distance = dot( v, v);

        assert( is_number( squared_distance ) || !"compute_closest_points(): NaN encountered");

        // Test that closest distance are non-increasing
        if(squared_distance > old_squared_distance)
        {
          // This means that the closest distance is increasing
          distance = sqrt( squared_distance );
          status = NON_DESCEND_DIRECTION;
          return;
        }

        // Test absolute stopping criteria
        if( squared_distance <= squared_absolute_tolerance )
        {
          // This basically means that p are so close to the
          // convex set that we consider the distance to be zero.
          distance = sqrt( squared_distance );
          status = ABSOLUTE_CONVERGENCE;
          return;
        }

        // Test relative stopping criteria, so see if we do not make enough
        // progress toward the ``solution''
        if( (old_squared_distance - squared_distance) <= (relative_tolerance*old_squared_distance) )
        {
          // If relative test succedes then it means that this is as good as it
          // gets and we consider the algorithm to have converged.
          distance = sqrt( squared_distance );
          status = RELATIVE_CONVERGENCE;
          return;
        }

        // Test for stagnation of the solution
        if(fabs( old_squared_distance - squared_distance ) <= stagnation_tolerance )
        {
          distance = sqrt( squared_distance );
          status = STAGNATION;
          return;
        }

      }

      // If this point of the code is reached it means that we did
      // not converge with the maximum number of iterations.
      distance = sqrt( squared_distance );
      status = EXCEEDED_MAX_ITERATIONS_LIMIT;
    }

    /**
    * Lazy Man's Version.
    * This function uses presets for parameters that control the algorithm.
    */
    template<
      typename transform_type
      , typename support_functor1
      , typename support_functor2
    >
    inline void compute_closest_points( 
      transform_type const & transform_A
    , support_functor1 const & A
    , transform_type const & transform_B
    , support_functor2 const & B
    , typename transform_type::vector3_type & p_a
    , typename transform_type::vector3_type & p_b
    , typename transform_type::value_type & distance
    , size_t & status
    )
    {
      typedef typename transform_type::value_type  T;

      OpenTissue::gjk::VoronoiSimplexSolverPolicy const simplex_solver_policy  = OpenTissue::gjk::VoronoiSimplexSolverPolicy();

      size_t       iterations           = 0u;
      size_t const max_iterations       = 100u;
      T      const absolute_tolerance   = boost::numeric_cast<T>(10e-6);
      T      const relative_tolerance   = boost::numeric_cast<T>(10e-10); // Don't be too greedy!
      T      const stagnation_tolerance = boost::numeric_cast<T>(10e-16);

      compute_closest_points( 
        transform_A
        , A
        , transform_B
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

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_COMPUTE_CLOSEST_POINTS_H
#endif
