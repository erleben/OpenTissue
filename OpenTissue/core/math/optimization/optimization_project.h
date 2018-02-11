#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECT_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>
#include <stdexcept>
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {


      /**
      * Project Operator.
      * Given the iterate x_k, this function performs the projection
      *
      *   x_{k+1} = P(x_k, l, u( )
      *
      * where l and u are considered to be lower and upper constant bounds of x.
      *
      * @param l The lower constant bounds we must have l <= u.
      * @param u The upper constant bounds we must have l <= u.      
      * @param x                     This argument holds the current value of the iterate.
      * @param new_x                 Upon return this argument holds the projected iterate.
      *
      * @return If a component of x is projected unto the corresponsing
      * component of either l or u, then the return value is true otherwise the
      * return value is false.
      */
      template < typename T >
      inline bool project(
          boost::numeric::ublas::vector<T> const & x
        , boost::numeric::ublas::vector<T> const & l
        , boost::numeric::ublas::vector<T> const & u
        , boost::numeric::ublas::vector<T> & new_x
        )  
      {
        size_t const m = x.size();

        if(x.size()<=0)
          throw std::invalid_argument("size of x-vector were zero");

        if(new_x.size()!=m)
          throw std::invalid_argument("size of new_x-vector were incompatible");

        if(l.size()!=m)
          throw std::invalid_argument("size of the x-vectors and lower bound were incompatible");

        if(u.size()!=m)
          throw std::invalid_argument("size of the x-vectors and upper bound were incompatible");

        bool changed = false;

        // Here we project x_k onto the box defined by l(x_k) and u(x_k)
        for (size_t i = 0; i < m; ++ i)
        {
          T const l_i = l(i);
          T const u_i = u(i);
          // Sanity test, whether the l and u functions return meaningfull values
          assert( is_number( l_i ) || !"project: l_i was not a number");
          assert( is_number( u_i ) || !"project: u_i was not a number");
          assert( l_i <= u_i || !"project(): inconsistent l and u values");
          T const x_i = x(i);
          assert( is_number( x_i ) || !"project: x_i was not a number");
          T const new_x_i = (x_i < l_i) ? l_i : ( ( x_i > u_i) ? u_i : x_i   );
          assert( is_number( new_x_i ) || !"project: new_x_i was not a number");
          changed = changed || new_x_i > x_i || new_x_i < x_i;
          new_x(i) = new_x_i;
        }
        return changed;
      }

      /**
      * Project Operator.
      * This function corresponds to calling the function, project(x,l,u,x);
      */
      template < typename T >
      inline bool project(
        boost::numeric::ublas::vector<T> & x
        , boost::numeric::ublas::vector<T> const & l
        , boost::numeric::ublas::vector<T> const & u
        )  
      {
        return OpenTissue::math::optimization::project(x,l,u,x);
      }

      /**
      * Project Operator.
      * Given the iterate x_k, this function performs the projection
      *
      *   x_{k+1} = P(x_k, l(x_k), u(x_k) )
      *
      * Since l and u are considered to be functions of x, it may well be that
      * if a component of x is projected then another componenet of x becomes
      * infeasible.
      *
      * However, if the return value of this function is false then one can be
      * sure that
      *
      *   x_{k+1} = P(x_{k+1}, l(x_{k+1}), u(x_{k+1}) )
      *
      * Thus one way around an infeasible iterate is to keep on iterating until
      * the function returns false. That is:
      *
      *   while( project( x, l, u , new_x) )
      *   {
      *     x = new_x;
      *   }
      *
      * This function do not know anything about the lower and upper bound
      * functions, l and u. That means that in general no guarantee can be
      * given that the above while-loop will terminate with a fix-point iterate
      * x. However, in most cases l and u are simple functions or constants, in
      * which case it should be relatively safe to perform the loop.
      *
      * @param l A bound function object type. This object represents the lower
      * bounds and must implement the method with the signature:
      *
      *    T const & () (ublas::vector<T> const & x, size_t const & i) const
      *
      * @param u A bound function object type (same type as l-parameter). This
      * object represents the upper bounds.
      *
      * @param x                     This argument holds the current value of the iterate.
      *
      * @param new_x                 Upon return this argument holds the projected iterate.
      *
      * @return If a component of x is projected unto the corresponsing
      * component of either l or u, then the return value is true otherwise the
      * return value is false.
      */
      template < typename T, typename bound_function_type>
      inline bool project(
        boost::numeric::ublas::vector<T> const & x
        , bound_function_type const & l
        , bound_function_type const & u
        , boost::numeric::ublas::vector<T> & new_x
        )  
      {
        size_t const m = x.size();

        if(x.size()<=0)
          throw std::invalid_argument("size of x-vector were zero");

        if(new_x.size()!=m)
          throw std::invalid_argument("size of new_x-vector were incompatible");

        bool changed = false;

        // Here we project x_k onto the box defined by l(x_k) and u(x_k)
        for (size_t i = 0; i < m; ++ i)
        {
          T const l_i = l(x,i);
          T const u_i = u(x,i);
          // Sanity test, whether the l and u functions return meaningfull values
          assert( is_number( l_i ) || !"project: l_i was not a number");
          assert( is_number( u_i ) || !"project: u_i was not a number");
          assert( l_i <= u_i || !"project(): inconsistent l and u values");
          T const x_i = x(i);
          assert( is_number( x_i ) || !"project: x_i was not a number");
          T const new_x_i = (x_i < l_i) ? l_i : ( ( x_i > u_i) ? u_i : x_i   );
          assert( is_number( new_x_i ) || !"project: new_x_i was not a number");
          changed = changed || new_x_i > x_i || new_x_i < x_i;
          new_x(i) = new_x_i;
        }
        return changed;
      }

      /**
      * Project Operator.
      * This function corresponds to calling the function, project(x,l,u,x);
      */
      template < typename T, typename bound_function_type>
      inline bool project(
        boost::numeric::ublas::vector<T> & x
        , bound_function_type const & l
        , bound_function_type const & u
        )  
      {
          // Unlike the function project(x,l,u,new_x) this function, project(x,l,u),
          // retrieves the most up to date bound values inside the
          // projection for-loop. Thus this function can be seen as a Gauss-Seidel like
          // way to perform the projection while the other function project(x,l,u,new_x)
          // is a Jacobi like projection.
        return OpenTissue::math::optimization::project(x,l,u,x);
      }

      /**
       * Projection Operator Class.
       * This class conveniently hides the lower and upper bounds
       * for the projection-function and wraps it all into one
       * single ``operator''.
       *
       * Intended usage is: 
       *
       *  vector_type x;
       *  bound_function_type l;
       *  bound_function_type u;
       *  Projection<T,bound_function_type> P(l,u);
       *  ... do something with x ....
       *  while(projection(x));
       *
       */
      template < typename T, typename bound_function_type>
      class Projection
      {
      public:

        bound_function_type const & m_l;
        bound_function_type const & m_u;

        Projection(
            bound_function_type const & l
          , bound_function_type const & u
          )
          : m_l(l)
          , m_u(u)
        {}

        bool operator()(boost::numeric::ublas::vector<T> & x) const 
        {  
          return OpenTissue::math::optimization::project(x,m_l,m_u); 
        }

        bool operator()(boost::numeric::ublas::vector<T> const & x, boost::numeric::ublas::vector<T> & x_new) const 
        {  
          return OpenTissue::math::optimization::project(x,m_l,m_u,x_new); 
        }

      };

      /**
       * Empty Projection Operator.
       * This operator corresponds to no-projection. It is 
       * usefull as a dummy argument for other functions when
       * one wants to ignore projections.
       */
      template < typename T >
      class NoProjection
      {
      public:
        bool operator()(boost::numeric::ublas::vector<T> &) const {return false;}
        bool operator()(boost::numeric::ublas::vector<T> const &,boost::numeric::ublas::vector<T>  &) const  { return false;}
      };


    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECT_H
#endif
