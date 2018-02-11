#ifndef OPENTISSUE_DYNAMICS_MBD_INTERFACES_MBD_NCP_SOLVER_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_INTERFACES_MBD_NCP_SOLVER_INTERFACE_H
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
  namespace mbd
  {

    template<  typename math_policy  >
    class NCPSolverInterface
    {
    protected:

      typedef typename math_policy::matrix_type         matrix_type;
      typedef typename math_policy::vector_type         vector_type;
      typedef typename math_policy::idx_vector_type     idx_vector_type;

    public:

      NCPSolverInterface(){}

      virtual ~NCPSolverInterface(){}

    public:

      /**
       * Run Sover.
       *
       * @param J       The Jacobian matrix, this is used to find the system matrix, A = J W J^T
       * @param W       The inverted mass matrix, this is used to find the system matrix, A = J W J^T
       * @param gamma   A regularization vector used to regularize the system such that the system matrix is replaced by, A' = A + diag(gamma).
       * @param b       This vector is termed the right-hand-side vector, used in setting up the relation, y = A x + b
       * @param lo      Lower bounds on the x-solution
       * @param hi      Upper bounds on the x-solution
       * @param pi      Some bounds are given as a linear mapping dependent on the x-solution, this vector encodes the linear dependency.
       * @param mu      For those bounds that are linear dependent this vector holds the coefficients.
       * @param x       Upon return this vector holds the solution 
       */
      virtual void run(
          matrix_type const & J
        , matrix_type const & W
        , vector_type const & gamma
        , vector_type const & b
        , vector_type & lo
        , vector_type & hi
        , idx_vector_type const & pi
        , vector_type const & mu
        , vector_type & x
        )  = 0;
    };

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_INTERFACES_MBD_NCP_SOLVER_INTERFACE_H
#endif
