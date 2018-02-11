#ifndef OPENTISSUE_CORE_MATH_MATH_RANDOM_H
#define OPENTISSUE_CORE_MATH_MATH_RANDOM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp> // Needed for boost::numeric_cast

#include <OpenTissue/core/math/math_constants.h>


#ifdef BOOST_VERSION  //--- FIXME: Nicer way too see if we have boost?
#  include <boost/random.hpp>
#else
#  include <ctime>   // for std::time()
#  include <cstdlib> // for std::srand() and std::rand(), defaults to this if no boost!
#endif

namespace OpenTissue
{

  namespace math
  {

#ifdef BOOST_RANDOM_HPP


    template<typename value_type>
    class Random
    {
    public:

      typedef boost::minstd_rand          generator_type;

    protected:

      static generator_type &  generator()
      {
        static generator_type tmp(static_cast<unsigned int>(std::time(0)));
        return tmp;
      }

    public:

      typedef value_type                                                     T;
      typedef boost::uniform_real<T>                                         distribution_type;
      typedef boost::variate_generator<generator_type&, distribution_type >  random_type;

      distribution_type       m_distribution;
      random_type             m_random;

    public:

      Random()
        : m_distribution(details<T>::zero(),details<T>::one())
        , m_random(generator(), m_distribution)
      {}

      Random(T lower,T upper)
        : m_distribution(lower,upper)
        , m_random(generator(), m_distribution)
      {}

    private:

      Random(Random const & rnd){}
      Random & operator=(Random const & rnd){return *this;}

    public:

      T operator()() { return m_random();  }

      bool operator==(Random const & rnd) const { return m_distribution == rnd.m_distribution; }

    };

#else


    template <typename value_type>
    class Random
    {
    protected:

      typedef value_type  T;
      typedef Random<T>   self;

      T m_lower;
      T m_upper;

    protected:

      static bool & is_initialized()
      {
        static bool initialized = false;
        return initialized;
      }

    public:

      Random() 
        : m_lower(math::detail::zero<T>()) 
        , m_upper(math::detail::one<T>())
      {
        using std::time;
        if(!is_initialized())
        {
          std::srand(static_cast<unsigned int>(std::time(0)));
          is_initialized() = true;
        }
      }

      Random(T lower,T upper) 
        : m_lower(lower) 
        , m_upper(upper)
      { 
        self();
      }

    private:

      Random(Random const & rnd){}
      Random & operator=(Random const & rnd){return *this;}

    public:

      T operator()() const
      {
        double rnd = rand()/(1.0*RAND_MAX);
        return boost::numeric_cast<T>(m_lower+(m_upper-m_lower)*rnd);
      }

      bool operator==(Random const & rnd) const { return (m_lower==rnd.m_lower && m_upper==rnd.m_upper);  }

    };

#endif

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_RANDOM_H
#endif
