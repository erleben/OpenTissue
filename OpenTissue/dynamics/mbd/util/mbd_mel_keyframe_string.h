#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_KEYFRAME_STRING_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_KEYFRAME_STRING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_euler_angles.h>

#include <string>
#include <sstream>


namespace OpenTissue
{
  namespace mbd
  {
    namespace mel
    {
      /**
      * MEL Keyframe String Tool.
      * Example usage:
      *
      * std::cout << mbd::mel::keyframe_string(configuration.body_begin(),configuration.body_end(),simulator.get_time())  << std::endl;
      *
      */
      template< typename indirect_body_iterator, typename real_type_>
      std::string keyframe_string(indirect_body_iterator begin,indirect_body_iterator end,real_type_ const & time)
      {
        typedef typename indirect_body_iterator::value_type   body_type;
        typedef typename body_type::vector3_type              vector3_type;
        typedef typename body_type::quaternion_type           quaternion_type;
        typedef typename body_type::matrix3x3_type            matrix3x3_type;
        typedef typename body_type::real_type                 real_type;
        typedef typename body_type::value_traits              value_traits;

        std::stringstream stream;

        for(indirect_body_iterator body=begin;body!=end;++body)
        {
          // TODO: Only works for SingleGroupAnalysis. Should be encapsulated in SFNIAE
          if(!body->is_active())
            continue;

          vector3_type r;
          body->get_position(r);
          real_type x = r(0);
          real_type y = r(1);
          real_type z = r(2);

          quaternion_type Q;
          body->get_orientation(Q);
          matrix3x3_type R(Q);

          real_type xangle = value_traits::zero();
          real_type yangle = value_traits::zero();
          real_type zangle = value_traits::zero();

          OpenTissue::math::euler_angles(R,xangle,yangle,zangle);

          xangle *= 180.0/value_traits::pi();  //--- Convert from radians into degrees
          yangle *= 180.0/value_traits::pi();
          zangle *= 180.0/value_traits::pi();

          stream << "move -a -ws  -xyz "
            << x
            << " "
            << y
            << " "
            << z
            << " body" << body->get_index()
            << ";"
            << std::endl;
          stream << "rotate -a "
            << xangle
            << " "
            << yangle
            << " "
            << zangle
            << " "
            << " body" << body->get_index()
            << ";"
            << std::endl;
          stream << "setKeyframe -t "
            << time
            << "sec "
            << " body" << body->get_index()
            << ";"
            << std::endl;
        }
        return stream.str();
      }

    } // namespace mel
  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_KEYFRAME_STRING_H
#endif
