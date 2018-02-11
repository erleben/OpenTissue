#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_CONSTRAINT_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_CONSTRAINT_INTERFACE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_core_constraint_interface.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Constraint base Class.
    * Constraints are always imposed between a pair of bodies. These bodies are
    * ordered uniquely such that the body denoted by A have the smallest index
    * and the body denoted by B have the greatest index.
    *
    * Besides that it defines several paramters common to all constraints such
    * as error reduction parameter and constraint force mixing.
    */
    template< typename mbd_types  >
    class ConstraintInterface : public CoreConstraintInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::body_type                       body_type;
      typedef typename mbd_types::math_policy::size_type           size_type;
      typedef typename mbd_types::math_policy::real_type           real_type;
      typedef typename mbd_types::math_policy::vector_range        vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range    idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range        matrix_range;

    protected:

      body_type   * m_bodyA;             ///< A pointer to the first body.
      body_type   * m_bodyB;             ///< A pointer to the second body.
      size_type     m_jacobian_idx;      ///< Starting row index if jacobian data in system jacobian (note that column indices are determined by bodies).

    public:

      ConstraintInterface()
        : m_bodyA(0)
        , m_bodyB(0)
        , m_jacobian_idx(0u)
      {}
      
      virtual ~ConstraintInterface(){}
      
    public:

      /**
      * Get body_type A.
      *
      * @return    A pointer to the first body.
      */
      body_type * get_body_A() const {  return m_bodyA; }

      /**
      * Get body_type B.
      *
      * @return    A pointer to the second body.
      */
      body_type * get_body_B() const {  return m_bodyB; }

      /**
      * Set Starting Index of Jacobian Data.
      *
      * @param idx    The global starting row index into the system jacobian
      *               and error term vector. That is the global index
      *               corresponding to local 0-row.
      */
      void set_jacobian_index(size_type const & idx)
      {
        assert(idx>=0u || !"ConstraintInterface::set_jacobian_idx(): idx must be non-negative");
        m_jacobian_idx = idx;
      }

      /**
      * Return Starting Index of Jacobian Data.
      *
      * @return    The starting row index into the system jacobian
      *            and error term vector.
      */
      size_type const & get_jacobian_index() const {  return m_jacobian_idx; }

    public:

      /**
      * Evaluate Constraint.
      * This method is responsible for evaluating the current state
      * of the constraint. This implies setting up any internal
      * data structures needed for construting the jacobians and
      * auxiliary data later on.
      *
      * Also the method is responsible for evaluating whetever the
      * constraint is actually active in the current state. This
      * implies that if the method isEvaluatedActive() is invoked afterwards
      * then it should return the active state determined by evaluate().
      */
      virtual void evaluate() = 0;

      /**
      * Get Linear Jacobian of body A.
      *
      * @param J  Upon return this matrix contains the request jacobian matrix. The
      *                matrix must have dimension mx3, where m is the number of jacobian
      *                variables (rows).
      */
      virtual void get_linear_jacobian_A(matrix_range & J) const = 0;

      /**
      * Get Linear Jacobian of body B.
      *
      * @param J  Upon return this matrix contains the request jacobian matrix. The
      *                matrix must have dimension mx3, where m is the number of jacobian
      *                variables (rows).
      */
      virtual void get_linear_jacobian_B(matrix_range & J) const = 0;

      /**
      * Get Angular Jacobian of body A.
      *
      * @param J  Upon return this matrix contains the request jacobian matrix. The
      *                matrix must have dimension mx3, where m is the number of jacobian
      *                variables (rows).
      */
      virtual void get_angular_jacobian_A(matrix_range & J) const = 0;

      /**
      * Get Angular Jacobian of body B.
      *
      * @param J  Upon return this matrix contains the request jacobian matrix. The
      *                matrix must have dimension mx3, where m is the number of jacobian
      *                variables (rows).
      */
      virtual void get_angular_jacobian_B(matrix_range & J) const = 0;

      /**
      * Get Error Term Vector.
      *
      * @param b_error  Upon return this vector holds the error term (or right
      *                 hand side vector) related to the jacobian variables.
      *                 Initially the vector must have the same size as the
      *                 number of jacobian variables (rows).
      */
      virtual void get_stabilization_term(vector_range & b_error) const = 0;

      /**
      * Get Jacobian Low Limits.
      *
      * @param lambda_low  Upon return this vector holds the low limits of
      *                    the lagrange multipliers associated with the jacobian
      *                    constraints.
      */
      virtual void get_low_limits(vector_range & lo) const = 0;

      /**
      * Get Jacobian High Limits.
      *
      * @param hi  Upon return this vector holds the high limits of
      *            the lagrange multipliers associated with the jacobian
      *            constraints. Must be have size equal to number of jacobian rows.
      */
      virtual void get_high_limits(vector_range & hi) const = 0;

      /**
      * Get Dependency Indices.
      * As an example say a constraint have tree variables, ie. three jacobian
      * rows, these are numbered locally 0,1, and 2. Imagine that the bounds
      * on the last two are dependent on the first row. Further assume that the
      * global index of the first row is given by the index-value m. In this
      * case the dep-vector should be equal to
      *
      *   [max_value,m,m]
      *
      * Note: An entry containing the maximum possible value of the underlying data
      * type indicates that there are no dependencies on the correspoding constraint.
      *
      * Note: The method should return global indices of the dependent
      * constraints and NOT local indices. One can use the method get_jacobian_index() to
      * retrieve the global index of the first constraint variable.
      *
      * @param dep    Upon return vector holds relative dependency
      *               indices. Must be have size equal to number of
      *               jacobian rows
      */
      virtual void get_dependency_indices(idx_vector_range & dep) const = 0;

      /**
      * Get Bounds Scaling Factor.
      * Imagine having the same case as described in the getDepedencyIndices() method.
      * The first row is not dependent on the others, so its scale factor is a ``don't
      * care'', if the bounds of the second row are twice the value of the first variable,
      * and the bounds on the third row are three times the value of the first
      * variable, then the fac-vector is equal to [0,2,3] real_type.
      *
      * Notice that all factor values are non-negative.
      *
      * @param factors    Upon return the vector holds all factor values.
      *                   Must be have size equal to number of jacobian
      *                   rows.
      */
      virtual void get_dependency_factors(vector_range & factors) const = 0;

      /**
      * Set Constraint force_type Mixing Parameters.
      * Note all values must be in the interval 0..1. The vector must have same
      * size as the number of jacobian constraints (rows in the jacobian
      * matrices).
      *
      * @param gamma   A vector containing the new parameter values.
      *                   Must be have size equal to number of jacobian
      *                   rows.
      */
      virtual void set_regularization(vector_range const & gamma) = 0;

      /**
      * Get Constraint force_type Mixing Vector.
      *
      * @param gamma   Upon return this vector holds the values of the gamma paramters.
      *                   Must be have size equal to number of jacobian
      *                   rows.
      */
      virtual void get_regularization(vector_range & gamma) const = 0;

      /**
      * Save Solution of Jacobian Lagrange Multipliers
      *
      * @param solution   A vector expression containing the solution to be saved.
      *                   Must be have size equal to number of jacobian
      *                   rows.
      */
      virtual void set_solution(vector_range const & solution) = 0;

      /**
      * Get Cached Solution.
      *
      * @param solution   Upon return this vector expression will contain the last stored solution.
      *                   Must be have size equal to number of jacobian
      *                   rows.
      */
      virtual void get_solution(vector_range & solution) const = 0;

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_CONSTRAINT_INTERFACE_H
#endif
