#ifndef OPENTISSUE_CORE_MATH_INTERVAL_IO_BOOST_INTERVAL_IO_H
#define OPENTISSUE_CORE_MATH_INTERVAL_IO_BOOST_INTERVAL_IO_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/numeric/interval.hpp>

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
      template<class T, class Policies, class CharType, class CharTraits>
      std::basic_ostream<CharType, CharTraits> & operator<<(
        std::basic_ostream<CharType, CharTraits> &stream
        , boost::numeric::interval<T, Policies> const & value
        )
      {
        if (empty(value)) 
        {
          return stream << "[]";
        }
        else 
        {
          return stream << '[' << lower(value) << ',' << upper(value) << ']';
        }
      }

      /**
      * Stream Input Operator.
      *
      * @param in   The stream.
      * @param i    The interval that be input from the specified stream.
      *
      * @return    A reference to the stream.
      */
      template<class T, class Policies, class CharType, class CharTraits>
      std::basic_istream<CharType, CharTraits> & operator>>(
        std::basic_istream<CharType, CharTraits> & stream
        , boost::numeric::interval<T, Policies> & value
        )
      {
        T l, u;
        char c = 0;
        stream >> c;
        if (c == '[') 
        {
          stream >> l >> c;
          if (c == ',')
            stream >> u >> c; 
          else 
            u = l;
          if (c != ']')
            stream.setstate(stream.failbit);
        }
        else 
        {
          stream.putback(c);
          stream >> l;
          u = l;
        }
        if (stream)
          value.assign(l, u);
        else
          value = boost::numeric::interval<T, Policies>::empty();
        return stream;
      }


    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_IO_BOOST_INTERVAL_IO_H
#endif 
