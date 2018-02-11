#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_MAKE_MBD_BOUNDS_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_MAKE_MBD_BOUNDS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <cmath>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      namespace detail
      {


        /**
        * Multibody Dynamics Bound Functor Wrapper.
        * This is a convenience class making it easier to construct bound
        * functor objects for a multibody constraint formulation.
        *
        * In Multibody Dynamics the constraint forces is easily
        * formulated as a linear mixed complementrarity problem (LMCP).
        *
        * The lower and upper bounds functions are in this case easily
        * computed from an index dependency vector, pi, and a coefficient vector, mu.
        *
        * pi:   This vector holds the indices of the dependent constraints.
        *             For some variables their upper and lower bounds are modelled
        *             as a linear dependency on some other variable.
        *             That is for the $j$'th variable we might have
        *
        *               lower_j = -mu_j x_i
        *               upper_j =  mu_j x_i
        *
        *             This vector holds the indices of the depedent variables. That is
        *
        *               pi_j = i
        *
        *            If the j'th entry stores the maximum possible value of the underlying
        *            data-type in the vector then it means that there are no dependencies.
        *
        * mu:  This vectors holds the values of the linear scaling factor of the dependent constraints.
        */
        template <typename T>
        class MultibodyDynamicsBoundFunctor
        {
        public:

          typedef T value_type;
          typedef OpenTissue::math::ValueTraits<T> value_traits;

        protected:

          bool                          m_is_lower;  ///< This boolean value indicates whether the functor represents lower bounds or upper bounds.
          ublas::vector<size_t> const & m_pi;        ///< Dependency index vector. This vector holds information about the coupling between tangential friction forces and the corresponding normal forces.
          ublas::vector<T>      const & m_mu;        ///< Friction coefficient vector. The i'th entry holds the friction coefficient that should be used for the i'th ``force''. Note that the vector also holds values for normal forces. However, these values are ``don't cares'' and are ignored in this functor.
          ublas::vector<T>      const & m_val;       ///< If there is no dependent variable (not a friction force) then the bound value is constant and is taken from this value vector.
          size_t                        m_n;         ///< The number of constraints (ie. the dimension of the bound function).

        public:

          /**
          * This vector iterator is used to iterate through all the non-zero partial derivatives of a constraint.
          */
          class vector_iterator
          {
          protected:

            bool   m_end;   ///< Boolean flag indicating whether this iterator is the end, i.e. one past the last element.
            size_t m_idx;   ///< The variable index, corresponding to the friction constraint who's partial derivatives of the bound function we are iterating.
            T      m_mu;    ///< Fortunately, there is only one non-zero partial derivative for a friction constraint, so we only need to store one value.

          public:

            vector_iterator()
              : m_end(true)
              , m_idx(0)
              , m_mu(value_traits::zero())
            {}

            vector_iterator(vector_iterator const & i) { *this = i; }

            vector_iterator(size_t const & idx, T const & mu)
              : m_end(false)
              , m_idx(idx)
              , m_mu(mu)
            {}

            bool const operator==(vector_iterator const & i) const     {      return this->m_end == i.m_end;     }
            bool const operator!=(vector_iterator const & i) const {  return !( (*this) == i); }

            size_t const index() const { return m_idx; }

            T operator*() const { return m_mu; }

            vector_iterator & operator=(vector_iterator const & i) 
            {
              this->m_end = i.m_end;
              this->m_idx = i.m_idx;
              this->m_mu  = i.m_mu;
              return *this;
            }

            vector_iterator const & operator++() 
            {
              m_end = true;
              return *this; 
            }
          };

        public:

          MultibodyDynamicsBoundFunctor(
            bool const & is_lower
            , ublas::vector<size_t> const & pi
            , ublas::vector<T> const & mu
            , ublas::vector<T> const & val
            )
            : m_is_lower(is_lower)
            , m_pi(pi)
            , m_mu(mu)
            , m_val(val)
            , m_n( pi.size() )
          {}

          template<typename vector_type>
          T operator()(vector_type const & x, size_t const & i) const
          {
            size_t const j = m_pi(i);

            if(j<m_n)
            {
              T  const mu_i = m_mu(i);
              return m_is_lower ? -mu_i*x(j) : mu_i*x(j);
            }

            return m_val(i);
          }

          vector_iterator partial_begin(size_t const & i) const 
          {
            size_t const j = m_pi(i);
            if(j<m_n)
            {
              T const  mu_i = m_mu(i);
              return m_is_lower ? vector_iterator(j,-mu_i) :  vector_iterator(j, mu_i);
            }
            return vector_iterator();
          }

          vector_iterator partial_end(size_t const & i) const {  return vector_iterator(); }

          size_t size() const { return m_n; }

        };

      } // namespace detail

      /**
      * Lower Bound Functor Factory Function.
      * This function creates lower bound functor for multibody dynamics formulation.
      *
      *
      * @param pi   This vector holds the indices of the dependent constraints.
      *             For some variables their upper and lower bounds are modelled
      *             as a linear dependency on some other variable.
      *             That is for the $j$'th variable we might have
      *
      *               lower_j = -mu_j x_i
      *               upper_j =  mu_j x_i
      *
      *             This vector holds the indices of the depedent variables. That is
      *
      *               pi_j = i
      *
      *            If the j'th entry stores the maximum possible value of the underlying
      *            data-type in the vector then it means that there are no dependencies.
      *
      * @param mu  This vectors holds the values of the linear scaling factor of the dependent constraints.
      * @param val In case there is no dependent index (pi_j >= n, where n is number of variables) then
      *            the constant value stored in this vector is used instead as the bound value.
      *
      * @return  A functor object, representing the lower bound function.
      *
      */
      template<typename T>
      inline detail::MultibodyDynamicsBoundFunctor<T> make_lower_mbd_bounds( 
          ublas::vector<size_t> const & pi
        , ublas::vector<T> const & mu
        , ublas::vector<T> const & val
        )
      {
        return detail::MultibodyDynamicsBoundFunctor<T>(true, pi, mu, val );
      }

      /**
      * Upper Bound Functor Factory Function.
      * This function creates upper bound functor for multibody dynamics formulation.
      *
      * @param pi   This vector holds the indices of the dependent constraints.
      *             For some variables their upper and lower bounds are modelled
      *             as a linear dependency on some other variable.
      *             That is for the $j$'th variable we might have
      *
      *               lower_j = -mu_j x_i
      *               upper_j =  mu_j x_i
      *
      *             This vector holds the indices of the depedent variables. That is
      *
      *               pi_j = i
      *
      *            If the j'th entry stores the maximum possible value of the underlying
      *            data-type in the vector then it means that there are no dependencies.
      *
      * @param mu  This vectors holds the values of the linear scaling factor of the dependent constraints.
      * @param val In case there is no dependent index (pi_j >= n, where n is number of variables) then
      *            the constant value stored in this vector is used instead as the bound value.
      *
      * @return  A functor object, representing the upper bound function.
      *
      */
      template<typename T>
      inline detail::MultibodyDynamicsBoundFunctor<T> make_upper_mbd_bounds( 
        ublas::vector<size_t> const & pi
        , ublas::vector<T> const & mu
        , ublas::vector<T> const & val
        )
      {
        return detail::MultibodyDynamicsBoundFunctor<T>(false, pi, mu, val );
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_MAKE_MBD_BOUNDS_H
#endif
