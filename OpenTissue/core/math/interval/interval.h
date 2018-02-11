#ifndef OPENTISSUE_CORE_MATH_INTERVAL_H
#define OPENTISSUE_CORE_MATH_INTERVAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interval/io/interval_io.h>
#include <OpenTissue/core/math/interval/interval_vector.h>
#include <OpenTissue/core/math/interval/interval_matrix.h>
#include <OpenTissue/core/math/interval/interval_intersect.h>
#include <OpenTissue/core/math/interval/interval_empty.h>

#include <OpenTissue/core/math/math_value_traits.h>


#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace interval
    {

      /**
      * Interval class.
      *
      * A *simple naive* speed test, no -DNDEBUG used.
      *
      * Intervals of type [x,y] where x<0 and y>0 used in testing.
      *
      *    this interval, 10^6 muls: 2.85913 seconds
      *    boost interval, 10^6 muls: 5.13671 seconds
      *
      *    this interval, 10^6 adds: 1.94612 seconds
      *    boost interval, 10^6 adds: 3.86221 seconds
      *
      *    this interval, 10^6 subs: 2.00829 seconds
      *    boost interval, 10^6 subs: 3.8229 seconds
      *
      * Intervals of type [x,y] where 0<x<y used in divs.
      *
      *   this interval, 10^6 divs: 3.41608 seconds
      *   boost interval, 10^6 divs: 4.78257 seconds
      *
      * This class contains >no< error-checking.
      * You might want to use boost::numeric::interval
      * until you know your interval-code is working.
      *
      * @author Jackj.
      */
      template <typename value_type_>
      class Interval
      {
      public:

        typedef value_type_                    T;
        typedef typename math::ValueTraits<T>  value_traits;
        typedef T                              value_type;
        typedef T                              base_type;   //--- boost::interval interface compatibility
        typedef size_t                         index_type;

      protected:

        T m_lower;  ///< The lower value of the interval. Default value is zero.
        T m_upper;  ///< The upper value of the interval. Default value is zero.

      public:

        T       & lower()       { return m_lower; }
        T const & lower() const { return m_lower; }
        T       & upper()       { return m_upper; }
        T const & upper() const { return m_upper; }

        /**
        * Interval Bound Operator Accessor
        *
        * @param     The index of the interval bound one want to access. Use zero
        *            to get lower bound and one to get upper bound.
        *
        * @return    The interval bound corresponding to the specified index.
        */
        T & operator() (index_type index) 
        {
          assert(index==0u || index==1u || !"interval::operator(): index must be zero or one");
          return (!index) ? m_lower : m_upper;
        }

        /**
        * Interval Bound Operator Accessor
        *
        * @param     The index of the interval bound one want to access. Use zero
        *            to get lower bound and one to get upper bound.
        *
        * @return    The interval bound corresponding to the specified index.
        */
        T const & operator() (index_type index) const
        {
          assert(index==0u || index==1u || !"interval::operator() const : index must be zero or one");
          return (!index) ? m_lower : m_upper;
        }

        /**
        * Interval Bound Operator Accessor
        *
        * @param     The index of the interval bound one want to access. Use zero
        *            to get lower bound and one to get upper bound.
        *
        * @return    The interval bound corresponding to the specified index.
        */
        T & operator[] (index_type index)
        {
          assert(index==0u || index==1u || !"interval::operator[]: index must be zero or one");
          return (!index) ? m_lower : m_upper;
        }

        /**
        * Interval Bound Operator Accessor
        *
        * @param     The index of the interval bound one want to access. Use zero
        *            to get lower bound and one to get upper bound.
        *
        * @return    The interval bound corresponding to the specified index.
        */
        T const & operator[] (index_type index) const
        {
          assert(index==0u || index==1u || !"interval::operator[] const : index must be zero or one");
          return (!index) ? m_lower : m_upper;
        }

      public:

        Interval()
          : m_lower()
          , m_upper()
        {}

        Interval(Interval const &i)
          : m_lower( i.m_lower )
          , m_upper( i.m_upper )
        {}

        explicit Interval(T const & value)
          : m_lower( value )
          , m_upper( value ) 
        {}

        explicit Interval(T const & l, T const & u)
          : m_lower( l )
          , m_upper( u ) 
        {
          //assert(lower_val <= upper_val || !"interval(lower_val,upper_val): lower bound must be less than or equal to upper bound");
        }

        ~Interval() {}

      public:

        /**
        * Assignment Operator.
        *
        * ToDo jackj: should this be explicit type conversrion?
        *
        *
        * @param rhs    The interval whos bounds are assigned to this interval.
        *
        * @return       A reference to this interval instance.
        */
        Interval & operator=(Interval const &rhs) 
        {
          m_lower =  rhs.m_lower;
          m_upper =  rhs.m_upper;
          return (*this);
        }

        //--- boost::interval interface compatibility
        void assign(T const &l, T const &u)
        {
          m_lower = l;
          m_upper = u;
        }

        /**
        * Clear Method.
        * This method sets the interval bounds to their default values.
        * I.e. both lower and upper bound is set to zero.
        */
        void clear()
        {
          m_lower = m_upper = value_traits::zero();
        }

        /**
        * Validity.
        * This method tests whether the bounds indicate a valid interval
        * range. This is useful for making integrity tests.
        */
        bool is_valid() const     {      return !is_empty();    }

        /**
        * Empty Test.
        * This method tests whether the bounds indicate an empty interval.
        *
        * By convention this means the lower bound is larger than the upper bound.
        */
        bool is_empty() const     {      return m_lower > m_upper;    }

        //--- boost::interval interface compatibility
        static Interval const & empty() 
        { 
          static Interval const empty_value(value_traits::one(),-value_traits::one()); 
          return empty_value;
        };

      public:

        /**
        * Interval Comparison Operators.
        * The following operators are implemented as they are in
        * interval aritchmetic, see for example (18) in
        * "Continuous Collision Detection for Rigid and Articulated Bodies"
        * by Stephane Redon
        *
        * or
        *
        * (2)-(6) in
        * "Interval Computations: Introduction, Uses, and Resources"
        * by R.B. Kearfott
        *
        * @param i  A reference to an interval for comparison.
        *
        * @return   The result of the comparison.
        */
        bool operator< (Interval const & i) const { return m_upper < i.m_lower;                  }
        bool operator> (Interval const & i) const { return m_lower > i.m_upper;                  }
        bool operator<=(Interval const & i) const { return m_upper <= i.m_lower;                 }
        bool operator>=(Interval const & i) const { return m_lower >= i.m_upper;                 }
        bool operator==(Interval const & i) const { return m_lower == i.m_lower && m_upper == i.m_upper; }
        bool operator!=(Interval const & i) const { return m_lower != i.m_lower || m_upper != i.m_upper; }

      public:

        /**
        * Interval Addition.
        *
        * @param     The interval that should be added to this interval.
        *
        * @return    A refrence to the resulting interval (this interval instance).
        */
        Interval & operator+=(Interval const & i)
        {
          m_lower += i.m_lower;
          m_upper += i.m_upper;
          return (*this);
        }

        /**
        * Interval Addition.
        * Performance Warning: Copy Constructor is invoked to create return value.
        *
        * @param     The interval that should be added to this interval.
        *
        * @return    A refrence to the resulting interval (this interval instance).
        */
        Interval operator+(Interval const &i ) const { return Interval(m_lower+i.m_lower, m_upper+i.m_upper); }

        /**
        * Interval Subtraction.
        *
        * @param     The interval that should be subtracted to this interval.
        *
        * @return    A refrence to the resulting interval (this interval instance).
        */
        Interval & operator-=(Interval const & i)
        {
          m_lower -= i.m_upper;
          m_upper -= i.m_lower;
          return (*this);
        }

        /**
        * Interval Subtraction.
        * Performance Warning: Copy Constructor is invoked to create return value.
        *
        * @param     The interval that should be subtracted to this interval.
        *
        * @return    A refrence to the resulting interval (this interval instance).
        */
        Interval operator-(Interval const & i) const { return Interval(m_lower-i.m_upper, m_upper-i.m_lower); }

        /**
        * Negation Operator.
        * Performance Warning: Copy Constructor is invoked to create return value.
        *
        * @return    An interval that holds the negated bounds of this interval reference.
        */
        Interval operator-() const {  return Interval(-m_upper, -m_lower); }

        /**
        * Scalar Multiplication.
        *
        * @param val   The scalar value to multiply this interval instance with.
        *
        * @return      A reference to this interval reference.
        */
        Interval & operator*=(T const & val)
        {
          if (val < value_traits::zero() ) 
          {
            T x = m_lower;
            m_lower = m_upper * val;
            m_upper = x * val;
          }
          else 
          {
            m_upper *= val;
            m_lower *= val;
          }
          return (*this);
        }

        /**
        * Scalar Multiplication.
        * Performance Warning: Copy Constructor is invoked to create return value.
        *
        * @param val   The scalar value to multiply this interval instance with.
        *
        * @return      A reference to this interval reference.
        */
        Interval operator*(T const & val) const
        {
          return (val < value_traits::zero()) ? Interval(m_upper*val, m_lower*val) : Interval(m_lower*val, m_upper*val);
        }

        /**
        * Interval Multiplication.
        *
        * @param i  The interval to multiply with.
        *
        * @return   A reference to this interval instance.
        */
        Interval & operator*=(Interval const & i)
        {
          T tmp_lower;
          T tmp_upper;

          if (m_lower <= value_traits::zero() && i.m_upper >= value_traits::zero()) 
          {
            tmp_lower = m_lower * i.m_upper;
          }
          else if (m_upper >= value_traits::zero()) 
          {
            tmp_lower = (i.m_lower <= value_traits::zero()) ? m_upper*i.m_lower : m_lower*i.m_lower;
          }
          else 
          {
            tmp_lower = m_upper*i.m_upper;
          }

          if (m_lower <= value_traits::zero() && i.m_lower <= value_traits::zero()) 
          {
            tmp_upper = m_lower * i.m_lower;
          }
          else if (m_upper >= value_traits::zero()) 
          {
            tmp_upper = (i.m_upper >= value_traits::zero())  ? m_upper * i.m_upper : m_lower * i.m_upper;
          }
          else 
          {
            tmp_upper = m_upper * i.m_lower;
          }

          m_upper = tmp_upper;
          m_lower = tmp_lower;

          return *this;
        }

        /*
        * Interval Multiplication.
        * Performance Warning: Copy Constructor is invoked to create return value.
        *
        * Optimized version, more IFs, two less mul
        *
        * @param i  The interval to multiply with.
        *
        * @return   A new interval instance holding the result of the multiplication.
        */
        Interval operator*(Interval const & i) const   
        {    
          Interval tmp(i);
          tmp *= (*this);
          return tmp;
        }

        /**
        * Interval Division.
        *
        * @param i   The interval to divided by.
        *
        * @return    A reference to this interval instance holding the result of the division.
        */
        Interval & operator/=(Interval const & i)
        {
          assert(i.m_lower > value_traits::zero() || i.m_upper < value_traits::zero() || !"interval::operator/=(): lower must be positive or upper negative!");
          (*this) *= Interval(value_traits::one() / i.m_upper, value_traits::one() / i.m_lower);
          return (*this);
        }

        /**
        * Interval Division.
        * Performance Warning: Copy Constructor is invoked to create return value.
        *
        * @param i   The interval to divided by.
        *
        * @return    A new interval instance holding the result of the division.
        */
        Interval operator/(Interval const & i) const
        {
          assert(i.m_lower > value_traits::zero() || i.m_upper < value_traits::zero() || !"interval::operator/=(): lower must be positive or upper negative!");
          return (*this) * Interval(value_traits::one() / i.m_upper, value_traits::one() / i.m_lower);
        }

        /**
        * Get Absolute Lower Interval Bound.
        *
        * @return   The value of the absolute lower interval bound.
        */
        T get_abs_lower() const
        {
          // -------0-[   ]- <-R
          // -----[ 0 ]----- <-R
          // -[   ]-0------- <-R
          return (m_lower >= value_traits::zero()) ? m_lower  :  ( m_upper >= value_traits::zero() ? value_traits::zero() : -m_upper  );
        }

        /**
        * Get Absolute Upper Interval Bound.
        *
        * @return   The value of the absolute upper interval bound.
        */
        T get_abs_upper() const
        {
          // -------0-[   ]- <-R
          // -----[ 0 ]----- <-R
          // -[   ]-0------- <-R
          return (m_lower+m_upper >= value_traits::zero()) ? m_upper : -m_lower;
        }

      };

    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_H
#endif 
