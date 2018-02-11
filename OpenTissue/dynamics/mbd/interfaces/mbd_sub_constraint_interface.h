#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_SUB_CONSTRAINT_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_SUB_CONSTRAINT_INTERFACE_H
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
    * Sub Constraint base Class.
    * Sub constraints are ``extra'' constrains on constraints between body pairs.
    *
    * For instance an joint limit or motor is an extra constraint on a hinge joint.
    *
    * These kind of constraints are termed sub-constraints, because they are subsequent
    * constraints on a ``base'' constraint.
    */
    template< typename mbd_types  >
    class SubConstraintInterface : public CoreConstraintInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy::size_type           size_type;
      typedef typename mbd_types::math_policy::real_type           real_type;
      typedef typename mbd_types::math_policy::vector_range        vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range    idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range        matrix_range;

    public:

      SubConstraintInterface(){}
      virtual ~SubConstraintInterface(){}

    public:

      virtual void get_linear_jacobian_A(matrix_range & J,size_type const & offset) const = 0;
      virtual void get_linear_jacobian_B(matrix_range & J,size_type const & offset) const = 0;
      virtual void get_angular_jacobian_A(matrix_range & J,size_type const & offset) const = 0;
      virtual void get_angular_jacobian_B(matrix_range & J,size_type const & offset) const = 0;
      virtual void get_stabilization_term(vector_range & b_error,size_type const & offset) const = 0;
      virtual void get_low_limits(vector_range & lo,size_type const & offset) const = 0;
      virtual void get_high_limits(vector_range & hi,size_type const & offset) const = 0;
      virtual void get_dependency_indices(idx_vector_range & dep,size_type const & offset) const = 0;
      virtual void get_dependency_factors(vector_range & factors,size_type const & offset) const = 0;
      virtual void set_regularization(vector_range const & gamma,size_type const & offset) = 0;
      virtual void get_regularization(vector_range & gamma,size_type const & offset) const = 0;
      virtual void set_solution(vector_range const & solution,size_type const & offset) = 0;
      virtual void get_solution(vector_range & solution,size_type const & offset) const = 0;

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_SUB_CONSTRAINT_INTERFACE_H
#endif
