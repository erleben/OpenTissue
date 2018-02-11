#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_MAKE_CONSTANT_BOUNDS_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_MAKE_CONSTANT_BOUNDS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {
      namespace detail
      {

        template <typename T>
        class ConstantVectorBoundFunctor
        {
        public:

          typedef T value_type;

        protected:

          ublas::vector<T>      const & m_values;

        public:

          /**
          * This vector iterator is used to iterate through all
          * the non-zero partial derivatives of a constraint.
          * However, for a constant vector there are no non-zero
          * partial derivatives.
          */
          class vector_iterator
          {
          public:

            vector_iterator(){}
            vector_iterator(vector_iterator const & i) { *this = i; }
            bool const operator==(vector_iterator const & i) const {      return true;     }
            bool const operator!=(vector_iterator const & i) const {  return !( (*this) == i); }
            size_t const index() const { return 0; }
            T operator*() const { return 0; }
            vector_iterator & operator=(vector_iterator const & i) { return *this; }
            vector_iterator const & operator++(){ return *this; }
          };

        public:

          ConstantVectorBoundFunctor(
            ublas::vector<T> const & values
            )
            : m_values(values)
          {}

          template<typename vector_type>
          T operator()(vector_type const & x, size_t const & i) const { return m_values(i); }
          vector_iterator partial_begin(size_t const & i) const { return vector_iterator(); }
          vector_iterator partial_end(size_t const & i) const {  return vector_iterator(); }

          size_t size() const { return m_values.size(); }
        };

        template <typename T>
        class ConstantValueBoundFunctor
        {
        public:

          typedef T value_type;

        protected:

          T m_value;
          size_t m_size;

        public:

          /**
          * This vector iterator is used to iterate through all
          * the non-zero partial derivatives of a constraint.
          * However, for a constant value there are no non-zero
          * partial derivatives.
          */
          class vector_iterator
          {
          public:

            vector_iterator(){}
            vector_iterator(vector_iterator const & i) { *this = i; }
            bool const operator==(vector_iterator const & i) const {      return true;     }
            bool const operator!=(vector_iterator const & i) const {  return !( (*this) == i); }
            size_t const index() const { return 0; }
            T operator*() const { return 0; }
            vector_iterator & operator=(vector_iterator const & i) { return *this; }
            vector_iterator const & operator++(){ return *this; }
          };

        public:

          ConstantValueBoundFunctor( T const & value, size_t const & size )
            : m_value(value)
            , m_size(size)
          {}

          template<typename vector_type>
          T operator()(vector_type const & x, size_t const & i) const { return m_value; }
          vector_iterator partial_begin(size_t const & i) const { return vector_iterator(); }
          vector_iterator partial_end(size_t const & i) const {  return vector_iterator(); }

          size_t size() const { return m_size; }
        };


      } // namespace detail

      /**
      * Constant Vector Bound Functor Factory Function.
      *
      * @param val  This vector holds the values of the constant bound function.
      *
      * @return  A functor object, representing the bound function.
      */
      template<typename T>
      inline detail::ConstantVectorBoundFunctor<T> make_constant_bounds( ublas::vector<T> const & val )
      {
        return detail::ConstantVectorBoundFunctor<T>( val );
      }

      /**
      * Constant Value Bound Functor Factory Function.
      *
      * @param value  The constant value.
      * @param size   The dimension of the bounds.
      *
      * @return  A functor object, representing the bound function.
      */
      template<typename T>
      inline detail::ConstantValueBoundFunctor<T> make_constant_bounds( T const & value, size_t const & size )
      {
        return detail::ConstantValueBoundFunctor<T>( value, size );
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_MAKE_CONSTANT_BOUNDS_H
#endif
