#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_BOUNDS2CONSTRAINTS_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_BOUNDS2CONSTRAINTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      namespace detail
      {
        /**
        * Bound to Constraint Conversion Functor.
        * In many cases one has bounded problems that is to say
        * that we have some problem subject to the constraints
        *
        *        x \geq l
        *        x \leq u
        *
        *  Or more generally the bounds could be dependent on x, that is
        *
        *        x \geq l(x)
        *        x \leq u(x)
        *
        * In some problems the bound functions l(x) and u(x) would be passed
        * directly to some solver. However, in other cases a solver or method
        * may work with a more general ``constraint'' function. That is we
        * can rewrite either the lower or upper bounds into a inequality
        * constraint of the type c(x) \geq 0,
        *
        *     c(x) =  u(x) - x \geq 0 or c(x) = x - l(x) \geq 0
        *
        * This is the purpose of the template class, it takes a bound function
        * and makes it behave as a constraint function.
        */
        template <typename T, typename bound_functor>
        class Bound2ConstraintFunctor
        {
        protected:

          typedef T value_type;
          typedef OpenTissue::math::ValueTraits<T> value_traits;
          typedef typename bound_functor::vector_iterator bound_iterator;

          bound_functor const & m_bounds;     ///< The bounds functor. This could be either a lower or an upper bound.
          bool m_is_lower;                    ///< Boolean flag, if set to true then the constraint functions corresponds to a lower bound otherwise the constraint function corresponds to an upper bound.

        public:

          Bound2ConstraintFunctor( 
            bound_functor const & bounds
            , bool const & is_lower 
            )
            : m_bounds(bounds)
            , m_is_lower(is_lower)
          {}

          template<typename vector_type>
          T operator()(vector_type const & x, size_t const & i) const 
          { 
            if (m_is_lower)
              return x(i) - m_bounds(x,i); 
            return m_bounds(x,i) - x(i); 
          }

          size_t size() const { return m_bounds.size(); }

        public:

          class vector_iterator
          {
          protected:

            size_t         m_index;     ///< The current index value of the iterator.
            size_t         m_end;       ///< The index value one position past the last element (if zero-based indexing is used, then this is the size of a vector).
            bound_iterator m_pos;       ///< A iterator to the ``native'' vector_iterator on the bound function.
            bool           m_is_lower;  ///< Boolean flag, if set to true then the constraint functions corresponds to a lower bound otherwise the constraint function corresponds to an upper bound.
            size_t         m_i;         ///< The variable index.

          public:

            vector_iterator(
                size_t const & begin
              , size_t const & end
              , bound_iterator & pos
              , bool const & is_lower
              , size_t const & i
              )
              : m_index(begin)
              , m_end(end)
              , m_pos(pos)
              , m_is_lower(is_lower)
              , m_i(i)
            {
              assert(begin <= end || !"vector_iterator: begin must be before end iterator!");
            }

            vector_iterator(vector_iterator const & i) { *this = i; }

            bool const operator==(vector_iterator const & i) const 
            {  
              return this->m_index == i.m_index; 
            }

            bool const operator!=(vector_iterator const & i) const 
            {
              return !( (*this) == i); 
            }

            size_t const index() const { return this->m_index; }

            T operator*() const 
            { 
              assert(m_index < m_end || !"vector_iterator: can not iterate pass the end");

              if (m_index == m_pos.index())
              {
                T const & bound = *(this->m_pos);
                if(m_index == m_i )
                  return (m_is_lower) ?  value_traits::one() - bound : bound - value_traits::one(); 
                else
                  return (m_is_lower) ?  - bound : bound; 
              }
              if(m_index == m_i )
                  return (m_is_lower) ?  value_traits::one() : - value_traits::one(); 

              return value_traits::zero() ;
            }

            vector_iterator & operator=(vector_iterator const & i) 
            {
              this->m_index    = i.m_index;
              this->m_end      = i.m_end;
              this->m_pos      = i.m_pos;
              this->m_is_lower = i.m_is_lower;
              return *this;
            }

            vector_iterator const & operator++() 
            {
              assert(m_index < m_end || !"vector_iterator: cannot increment pass the end iterator");
              if(m_index==m_pos.index())
                ++m_pos;
              ++m_index;
              return *this; 
            }
          };

          vector_iterator partial_begin(size_t const & i) const 
          { 
            size_t const end = this->size();
            size_t const begin = 0u;
            size_t const cur = i;
            bound_iterator it = m_bounds.partial_begin(i);
            return vector_iterator(begin, end, it, m_is_lower, cur ); 
          }

          vector_iterator partial_end(size_t const & i) const 
          {  
            size_t const end = this->size();
            size_t const cur = i;
            bound_iterator it = m_bounds.partial_end(i);
            return vector_iterator( end, end, it, m_is_lower, cur ); 
          }

        };

      } // namespace detail

      template<typename bound_functor>
      inline detail::Bound2ConstraintFunctor<typename bound_functor::value_type,bound_functor>
        make_constraint_from_lower_bounds(bound_functor const & lower)
      {
        return detail::Bound2ConstraintFunctor<typename bound_functor::value_type,bound_functor>(lower,true);
      }

      template<typename bound_functor>
      inline detail::Bound2ConstraintFunctor<typename bound_functor::value_type,bound_functor>
        make_constraint_from_upper_bounds(bound_functor const & upper)
      {
        return detail::Bound2ConstraintFunctor<typename bound_functor::value_type,bound_functor>(upper,false);
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_BOUNDS2CONSTRAINT_H
#endif
