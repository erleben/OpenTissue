#ifndef OPENTISSUE_CORE_MATH_INTERVAL_IO_INTERVAL_IO_H
#define OPENTISSUE_CORE_MATH_INTERVAL_IO_INTERVAL_IO_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interval/interval_fwd.h> //--- needed for forward declation of OpenTissue::interval
#include <iosfwd>

namespace OpenTissue
{
  namespace math
  {
    namespace interval
    {

      /**
      * Stream Output Operator.
      *
      * @param out   The stream.
      * @param i     The interval that should be output to a stream.
      *
      * @return    A reference to the stream.
      */
      template<typename T, class CharType, class CharTraits>
      std::basic_ostream<CharType,CharTraits> & operator<<(std::basic_ostream<CharType,CharTraits>  & out, Interval<T> const &i ) 
      {
        out << "["
          << i.lower() 
          << "," 
          << i.upper() 
          << "]";
        return out;
      }

      /**
      * Stream Input Operator.
      *
      * @param in   The stream.
      * @param i    The interval that be input from the specified stream.
      *
      * @return    A reference to the stream.
      */
      template<typename T, class CharType, class CharTraits>
      std::basic_istream<CharType,CharTraits> & operator>>(std::basic_istream<CharType,CharTraits> & in, Interval<T> const &i) 
      {
        char dummy;
        in >> dummy;
        in >> i.lower();
        in >> dummy;
        in >> i.upper();
        in >> dummy;
        return in;
      }

    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_IO_INTERVAL_IO_H

#endif 
