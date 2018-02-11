#ifndef OPENTISSUE_UTILITY_DISPATCHERS_DISPATCHERS_UNBOUND_DISPATCH_FUNCTION_H
#define OPENTISSUE_UTILITY_DISPATCHERS_DISPATCHERS_UNBOUND_DISPATCH_FUNCTION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace utility
  {
    namespace dispatchers
    {


      /** Exception thrown by the dispatcher.
      * This exception is thrown if the dispatcher have no dispatch function bound
      * to some specific combination of classes.
      * @see MultiDispatcher::operator()
      */
      class UnboundDispatchFunction
        : public std::runtime_error
      {
      public:

        /** Constructor
        *
        * @param[in] __arg
        *   Human readable text describing the exact problem.
        */
        explicit UnboundDispatchFunction(std::string const & __arg = "Unbound Dispatch Function for this combination")
          : std::runtime_error(__arg.c_str())
        {}

        /** Return a human readable description of the problem.
        * Mostly useful for logging and debugging.
        *
        * @return A human readable C-style string.
        */
        virtual char const * what() const throw()
        {
          return std::runtime_error::what();
        }
      };

    } // namespace dispatchers
  } // namespace utility
} // namespace OpenTissue

// OPENTISSUE_UTILITY_DISPATCHERS_DISPATCHERS_UNBOUND_DISPATCH_FUNCTION_H
#endif 
