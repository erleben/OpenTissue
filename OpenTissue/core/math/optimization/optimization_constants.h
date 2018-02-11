#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_CONSTANTS_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_CONSTANTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <string>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      /**
      * @file
      * This file contains declarations of some readable constants that is
      * used for setting proper error code/status before exiting any given
      * numerical method in the optimization library. The intention is that
      * this ensures that all numerical error codes have the same global meaning
      * regardless of the methods/functions that is used.
      *
      * @author Kenny Erleben
      */

      /**
      * No error occured.
      */
      size_t const OK = 0;

      /**
      * A non descent direction was encountered. This may occur
      * during a line-search method. In which case one expects
      * to perform a line-search along a descent direction.
      */
      size_t const NON_DESCENT_DIRECTION = 1;

      /**
      * A back-tracking operation failure was detected during some
      * line-search method. This often indicates that the step-length
      * have become too small and no decrease in (merit) function
      * value was obtained.
      */
      size_t const BACKTRACKING_FAILED = 2;

      /**
      * Stagnation occured.
      * Stagnation means that the maximum difference between the
      * components of a new iterate and the old iterate dropped
      * below some small threshold value. Basically it means
      * that the two iterates are nearly the same and no progress
      * have been made by the numerical method used.
      */
      size_t const STAGNATION = 3;

      /**
      * Relative Convergence Test Succeded.
      * This means that the relative improvement in function value has dropped below
      * a given threshold value. Given the current iterate \f$x^{n+1}\f$ and the
      * previous iterate\f$x^n\f$ then the test is
      *
      * \f[ \frac{| f(x^{n+1}) - f(x^n) |}{|f(x^n)|} < \varepsilon,  \f]
      *
      *where \f$\varepsilon \eq 0\f$ is a user specified test-threshold.
      */
      size_t const RELATIVE_CONVERGENCE = 4;

      /**
      * Absolute Convergence Test Succeded.
      * This means that the absolute function value has dropped below
      * a given threshold value. Given the current iterate \f$x^{n}\f$ then the test is
      *
      * \f[ f(x^n) < \varepsilon,  \f]
      *
      *where \f$\varepsilon \eq 0\f$ is a user specified test-threshold. In case
      * of minimization the test could also be for a stationary point. The test
      * would then be
      *
      * \f[ | \nabla f(x^n) | < \varepsilon,  \f]
      *
      */
      size_t const ABSOLUTE_CONVERGENCE = 5;

      /**
      * Halted while iterating.
      * This means that somehow for whatever unknown reason the method/function has
      * halted while iterating. In some cases this indicates some internal error. In
      * some cases this may occur if a method/function is being feed a bad (ill-posed
      * or inconsistent) problem to solve.
      *
      * However in most cases it simply means that the method did not converge within
      * some given maximum number of iterations.
      */
      size_t const ITERATING  = 6;



      /**
       * Descend Direction is in Normal Cone.
       * This means that the current iterate must be placed on the boundary of       
       * the feasible region and that the descend direction is point away from
       * the feasible region in the normal direction. Thus one can not even
       * slide along the boundary of the feasible region in order to minimize
       * ones function value.
       *
       * In many cases this means that the current iterate is as good as its
       * get (when considering local information only).
       */
      size_t const DESCEND_DIRECTION_IN_NORMAL_CONE  = 7;



      /**
      * Get Error Message.
      * This function decodes an given error code value
      * into a user friendly and human readable text string. The
      * text string may be convenient for displaying error
      * messages in a log or on screen for an end-user.
      *
      * @param error_code   The value of an error code.
      *
      * @return             A textual and human readable error message string.
      */
      inline std::string get_error_message(size_t const & error_code)
      {
        std::string msg;
        switch(error_code)
        {
        case OK:                               msg = "no error"; break;
        case NON_DESCENT_DIRECTION:            msg = "Non descent direction was encountered"; break;
        case BACKTRACKING_FAILED:              msg = "Back-tracking failure during line-search"; break;
        case STAGNATION:                       msg = "Stagnation test passed"; break;
        case RELATIVE_CONVERGENCE:             msg = "Relative convergence test passed"; break;
        case ABSOLUTE_CONVERGENCE:             msg = "Absolute convergence test passed"; break;
        case ITERATING:                        msg = "Halted while iterating or did not converge with maximum number of iterations"; break;
        case DESCEND_DIRECTION_IN_NORMAL_CONE: msg = "Descend Direction is in Normal Cone"; break;
        default:                               msg = "unrecognised error"; break;
        };
        return msg;
      }







      /**
      * In Lower Constraint Set Bitmask.
      * Given the general non-linear function, \f$y = f(x)\f$, and
      * the lower and upper bounds: \f$l\f$ and $\u\f$.
      * The non-linear mixed complementarity formulation can be stated as follows
      *
      *  \f[ y_i(x) < 0 \Rightarrow x_i = u_i \f]
      *  \f[ y_i(x) > 0 \Rightarrow x_i = l_i \f]
      *  \f[ y_i(x) = 0 \Rightarrow l_i \leq x_i \leq u_i  \f]
      *
      * The ``in lower'' bit is set if and only if  \f$y_i > (x_i - l_i)\f$.
      */
      size_t const IN_LOWER  = 1;

      /**
      * In Upper Constraint Set Bitmask.
      * Given the general non-linear function, \f$y = f(x)\f$, and
      * the lower and upper bounds: \f$l\f$ and $\u\f$.
      * The non-linear mixed complementarity formulation can be stated as follows
      *
      *  \f[ y_i(x) < 0 \Rightarrow x_i = u_i \f]
      *  \f[ y_i(x) > 0 \Rightarrow x_i = l_i \f]
      *  \f[ y_i(x) = 0 \Rightarrow l_i \leq x_i \leq u_i  \f]
      *
      * The ``in upper'' bit is set if and only if  \f$y_i < (x_i - u_i)\f$.
      */
      size_t const IN_UPPER  = 2;

      /**
      * In Active Constraint Set Bitmask.
      * Given the general non-linear function, \f$y = f(x)\f$, and
      * the lower and upper bounds: \f$l\f$ and $\u\f$.
      * The non-linear mixed complementarity formulation can be stated as follows
      *
      *  \f[ y_i(x) < 0 \Rightarrow x_i = u_i \f]
      *  \f[ y_i(x) > 0 \Rightarrow x_i = l_i \f]
      *  \f[ y_i(x) = 0 \Rightarrow l_i \leq x_i \leq u_i  \f]
      *
      * The ``in active'' bit is set if and only if  \f$y_i \leq (x_i - l_i)\f$ and \f$y_i \geq (x_i - u_i)\f$.
      */
      size_t const IN_ACTIVE = 4;


    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_CONSTANTS_H
#endif
