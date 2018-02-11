#ifndef OPENTISSUE_CORE_MATH_INTERVAL_BOOST_INTERVAL_TYPE_TRAITS_H
#define OPENTISSUE_CORE_MATH_INTERVAL_BOOST_INTERVAL_TYPE_TRAITS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interval/io/boost_interval_io.h>
#include <OpenTissue/core/math/interval/interval_vector.h>
#include <OpenTissue/core/math/interval/interval_matrix.h>

#include <boost/numeric/interval.hpp>

namespace OpenTissue
{
  namespace math
  {
    namespace interval
    {

      /**
      * Boost Interval Fixed Type Traits.
      * This class provides easy type handling of the boost interval class
      *
      *   http://www.boost.org/libs/numeric/interval/doc/interval.htm
      *
      * In computer graphics, we really do not need all the safety we
      * get from the default interval type:
      *
      *    typedef  ... real_type;
      *    typedef boost::numeric::interval<real_type>  interval_type;
      *
      * This class tweaks the policies of the interval class inorder to sacrifice
      * everything for the need for speed.
      *
      * According to 
      *
      *   http://www.boost.org/libs/numeric/interval/doc/rounding.htm#perf
      *
      * It reads:
      *
      *   ``for fast wide intervals without any rounding nor precision, use save_state_nothing<rounded_transc_exact<T>  >;''
      *
      * From
      *
      *   http://www.boost.org/libs/numeric/interval/doc/checking.htm
      *
      * We have:
      *
      *  ``If you do not mind having undefined results when an empty interval
      *    or an interval number is produced, your best bet is to create your
      *    own policy by overloading checking_base and modifying is_nan et is_empty
      *    in order for them to always return false. It is probably the fastest
      *    checking policy available; however, it suffers from its deficient
      *    security.''
      *
      * From these we have created our own rounding and checking policies. This
      * class provides an easy way to extract the interval type for this
      * ``speed optimized'' interval usage:
      *
      *  #include <OpenTissue/core/math/interval/interval_boost_interval_type_traits.h>
      *
      *  typedef BoostIntervalTypeTraits<double>::interval_type interval_type;  
      *
      *   ....
      *
      * Initial testings (see interval demo application) With .NET 2005 C++ compiler,
      * this fast boost interval type outruns the OpenTissue interval type by a factor
      * of 2. With gcc it seems that OpenTissue interval type is capable of
      * beating boost. However, this fast boost type comes pretty close to the
      * OpenTissue type on gcc.
      *
      * Our (Henrik Dohlann and Kenny Erleben) recommendations: Use this fast boost interval
      * type if you do not need the gcc-cutting edge.
      *
      *  Sometimes it is possible to add further optimizations by adding the
      *  following code around interval ``intensive blocks''
      *
      *    typedef .... interval_type;
      *    // save and initialize the rounding mode
      *    interval_type::traits_type::rounding rnd;
      *    // define the unprotected version of the interval type
      *    typedef  boost::numeric::interval_lib::unprotect<interval_type>::type interval_type2;
      *
      * Now one should use interval_type2 in the following computaiton block. In our test runs
      * this ``unprotect'' tweak did not add a lot of performance to the fast boost interval type
      * this class provides. Thus we recommend only using the ``unprotect''-way if the default
      * boost interval type is used.
      */
      template<typename T>
      class BoostIntervalTypeTraits
      {
      protected:

        template<typename T2>
        struct raw_checking : public boost::numeric::interval_lib::checking_base<T2>
        {
          static bool is_nan(const T2&) { return false; }
          static bool is_empty(const T2&, const T2&){ return false; }
        };

        typedef raw_checking<T> checking_type;
        typedef boost::numeric::interval_lib::save_state_nothing< boost::numeric::interval_lib::rounded_arith_exact<T> > rounding_type;
        typedef boost::numeric::interval_lib::policies< rounding_type, checking_type > policy_type;

      public:

        typedef boost::numeric::interval<T >                   default_interval_type;
        typedef boost::numeric::interval<T, policy_type >      interval_type;          ///< Fast interval type, but do not have any rounding, precision or checking policies worth of mentioning. If you use this then you should know what you are doing.

      };


    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_BOOST_INTERVAL_TYPE_TRAITS_H
#endif 
