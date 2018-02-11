#ifndef OPENTISSUE_COLLISION_GJK_GJK_CONSTANTS_H
#define OPENTISSUE_COLLISION_GJK_GJK_CONSTANTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <string>

/** 
* @file This file defines constants used in the GJK algorithm.
* The constants are used to tell callers about the exit status of
* the GJK algorithm. In this way a caller can see if GJK was succesfull
* or not. In cases where GJK is not successfull the caller can use the
* status information to take an appropriate counter action.
*
*/

namespace OpenTissue
{
  namespace gjk
  {
    /**
    * Absolute convergence status code.
    * This means that the absolute function value has dropped below
    * a given threshold value. Given the current iterate \f$x^{n}\f$ then the test is
    *
    * \f[ f(x^n) < \varepsilon,  \f]
    *
    *where \f$\varepsilon \eq 0\f$ is a user specified test-threshold.  In case
    * of minimization the test could also be for a stationary point. The test
    * would then be
    *
    * \f[ | \nabla f(x^n) | < \varepsilon,  \f]
    *
    */
    size_t const ABSOLUTE_CONVERGENCE = 0u;

    /**
    * Relative convergence status code.
    * This means that the relative improvement in function value has dropped below
    * a given threshold value. Given the current iterate \f$x^{n+1}\f$ and the
    * previous iterate\f$x^n\f$ then the test is
    *
    * \f[ \frac{| f(x^{n+1}) - f(x^n) |}{|f(x^n)|} < \varepsilon,  \f]
    *
    *where \f$\varepsilon \eq 0\f$ is a user specified test-threshold.
    */
    size_t const RELATIVE_CONVERGENCE = 1u;

    /**
    * Stagnation status code.
    * Stagnation means that the maximum difference between the
    * components of a new iterate and the old iterate dropped
    * below some small threshold value. Basically it means
    * that the two iterates are nearly the same and no progress
    * have been made by the numerical method used.
    */
    size_t const STAGNATION = 2u;

    /**
    * Intersection status code.
    */
    size_t const INTERSECTION = 3u;

    /**
    * Exceeded maximum iterations limit status code.
    */
    size_t const EXCEEDED_MAX_ITERATIONS_LIMIT = 4u;

    /**
    * Iterating status code.
    * This status code basically means that one got an unexpected
    * exit. It should never happen, but if it does this is a clear
    * indication of an internal error.
    */
    size_t const ITERATING = 5u;

    /**
    * Non-descend status code.
    * This status code is returned if an iteration is encountered
    * where the closest distance has increased.
    */
    size_t const NON_DESCEND_DIRECTION = 6u;


    /**
    * Simplex Expansion Failure.
    * This status code is returned if during an iteration no
    * new points can be added to the current simplex.
    */
    size_t const SIMPLEX_EXPANSION_FAILED = 7u;


    /**
     * Relative Convergene by Lower Error Bound.
     */
    size_t const LOWER_ERROR_BOUND_CONVERGENCE = 8u;

    /**
    * Get Status Message.
    * This function decodes an given status code value
    * into a user friendly and human readable text string. The
    * text string may be convenient for displaying status
    * messages in a log or on screen for an end-user.
    *
    * @param   code   The value of an error code.
    *
    * @return         A textual and human readable status message string.
    */
    inline std::string get_status_message(size_t const & code)
    {
      std::string msg;
      switch(code)
      {
      case ABSOLUTE_CONVERGENCE:             msg = "Absolute convergence test passed";          break;
      case RELATIVE_CONVERGENCE:             msg = "Relative convergence test passed";          break;
      case STAGNATION:                       msg = "Stagnation test passed";                    break;
      case INTERSECTION:                     msg = "Intersection was found";                    break;
      case EXCEEDED_MAX_ITERATIONS_LIMIT:    msg = "Maximum iteration limit was exceeded";      break;
      case ITERATING:                        msg = "Unexpected termination while iterating";    break;
      case NON_DESCEND_DIRECTION:            msg = "Non descent direction was encountered";     break;
      case SIMPLEX_EXPANSION_FAILED:         msg = "Simplex expansion failure";                 break;
      case LOWER_ERROR_BOUND_CONVERGENCE:    msg = "Relative convergence of lower error bound"; break;
      default:                               msg = "unrecognised error";                        break;
      };
      return msg;
    }

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_CONSTANTS_H
#endif
