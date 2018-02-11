#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_EULER_FILTER_STRING_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_EULER_FILTER_STRING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <string>
#include <sstream>

namespace OpenTissue
{
  namespace mbd
  {
    namespace mel
    {
      /**
      * MEL Euler Filter String Tool.
      * Must be applied when finished creating motion curves, othewise rotations will wrap around!!!
      *
      * Example usage:
      *
      * std::cout << mbd::mel::euler_filter_string(configuration.body_begin(),configuration.body_end())  << std::endl;
      */
      template< typename indirect_body_iterator>
      std::string euler_filter_string(indirect_body_iterator begin,indirect_body_iterator end)
      {
        std::stringstream stream;
        for(indirect_body_iterator body=begin;body!=end;++body)
        {
          if(body->is_active())
          {
            stream << std::endl;
            stream << "filterCurve body" << body->get_index() << ".rotateX body" << body->get_index() << ".rotateY body"<< body->get_index() <<".rotateZ;" << std::endl;
            stream << std::endl;
          }
        }
        return stream.str();
      }

    } // namespace mel

  } // namespace mbd

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_EULER_FILTER_STRING_H
#endif
