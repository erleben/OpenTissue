#ifndef OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_VARIATIONAL_INTERPOLATOR_H
#define OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_VARIATIONAL_INTERPOLATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_lu.h>

#include <boost/lambda/lambda.hpp>

#include <cmath>
#include <algorithm> // for copy
#include <iterator>  // for back_inserter

#ifndef NDEBUG
#include <fstream>
#endif

namespace OpenTissue
{
  namespace interpolation
  {

    namespace detail
    {

      /**
      * Radial Basis Function.
      *
      * @param x     The value at which to evaluate the radial basis function
      *
      * @return      The value of the radial basis-function at the point x.
      */
      template<typename vector_type>
      inline typename vector_type::value_type  radial_basis_function(vector_type const & x) 
      {
        using std::log;
        typedef typename vector_type::value_type  real_type;

        real_type dot_prod = x*x;
        real_type norm     = sqrt(dot_prod);
        if(norm>0)
          return dot_prod * log( norm );
        return norm;
      }


      /**
       * An implicit function.
       */
      template<typename small_vector_type>
      class ImplicitFunction
      {
      public:

        typedef typename small_vector_type::value_type  real_type;

        typedef typename ublas::matrix<real_type>       big_matrix_type;
        typedef typename ublas::vector<real_type>       big_vector_type;
        typedef typename big_matrix_type::size_type     size_type;

        typedef small_vector_type              return_type;

      protected:

        big_vector_type                  m_x;      ///< The vector containing the interpolation weights.
        big_vector_type                  m_b;      ///< Right hand side vector, holds pre-screibed implicit function values.
        big_matrix_type                  m_A;      ///< Constraint matrix.

        size_type                        m_N;      ///< Number of constraints
        size_type                        m_dim;    ///< The dimension of the scattered points that is being interpolated.
        size_type                        m_M;      ///< The size of the symmetric A-matrix.
        std::vector<small_vector_type>   m_c;      ///< Constraint points (boundary and normal mixed).

      public:

        /**
        *
        * @param point_begin
        * @param point_end
        * @param normal_begin
        * @param normal_end
        *
        */
        template<typename iterator>
          void init( iterator point_begin,iterator point_end, iterator normal_begin,iterator normal_end)
        {
          real_type   k = 0.01;  // TODO: KE 2006-0606 Should depend on geometry size! But I am lazy

          using std::distance;

          size_type  cnt_points  = distance(point_begin,point_end);
          size_type  cnt_normals = distance(normal_begin,normal_end);

          assert(cnt_points==cnt_normals || !"implicit_function::init(): The number of normals and points did not match");

          m_dim = point_begin->size();

          m_N  = cnt_points+cnt_normals;
          m_M  = m_N + m_dim + 1;

          std::copy(point_begin, point_end, std::back_inserter( m_c ) );

          // TODO: Rewrite into a one-liner?
          // hd 2006-06-09 - Using transform and boost::lambda, this is possible.
          //                 However, I couldn't convince boost to deduce the return type, 
          //                 so I had to help it with ret<small_vector_type> that looks
          //                 a bit clumsy.
          //                 Further, I don't know if we are allowed to use using namespace
          //                 like this without polluting the global namespace? Dropping the 
          //                 useing stuff, however, would make the one-liner even uglier...
          {
            using namespace boost::lambda;
            std::transform( point_begin, point_end, normal_begin, std::back_inserter( m_c ), 
              ret<small_vector_type>(_1 + ret<small_vector_type>(_2*k)) );
          }
          //iterator p = point_begin;
          //iterator n = normal_begin;
          //for(;n!=normal_end;++p,++n)
          //  m_c.push_back( (*p) + (*n)*k );

          m_x.resize( m_M );
          m_b.resize( m_M );
          m_A.resize( m_M, m_M );

          m_A.clear();
          m_b.clear();
          m_x.clear();

          //--- take care of b-vector
          for(unsigned int i=0;i<m_M;++i)
          {
            if(i<cnt_points)
              m_b(i) = 0;
            else if (i<m_N)
              m_b(i) = 1;
            else
              m_b(i) = 0;
          }
          //--- take care of phi-part
          for(unsigned int i=0;i<m_N;++i)
            for(unsigned int j=i;j<m_N;++j)
            {
              real_type phi_ij = radial_basis_function( m_c[i] - m_c[j] );
              m_A(i,j) = m_A(j,i)  = phi_ij;
            }
            //--- take care of polynomial part
            for(unsigned int i=0;i<m_N;++i)
            {
              m_A(i, m_N) = m_A(m_N , i) = 1;
              for(unsigned int j=0;j<m_dim;++j)
                m_A(i,j+m_N+1) = m_A(j+m_N+1,i)  = m_c[i](j);
            }
            math::big::lu(m_A,m_x,m_b);
        }

      public:

        /** 
        * Evaluate implicit function value.
        *
        * @param x        The point at which the implicit function should be evaluated.
        *
        * @return         The value of the implicit function.
        */
        real_type operator()( small_vector_type const & x ) const
        {
          real_type value = 0;

          for(unsigned int j=0;j<m_N;++j)
            value += m_x(j)*radial_basis_function(   x - m_c[j] );

          value += m_x(m_N);
          for(unsigned int j=0;j<m_dim;++j)
            value += m_x(m_N + j +1)*(x(j)); 

          return value;
        }

      };

    }// namespace detail


    /**
    * Variational Interpolation.
    * This implementation is based on the paper:
    *
    *  Greg Turk and James O'Brien, "Shape Transformation Using Variational Implicit Functions," SIGGRAPH 99, August 1999, pp. 335-342. 
    * 
    * This can be obtained from: http://www-static.cc.gatech.edu/~turk/my_papers/schange.pdf
    *
    * Intended usage:
    *
    *
    *   typedef ....  vector3_type;
    *   typedef ...   grid_type;
    *
    *   points[0] = vector3_type(  0.008337971754, -0.393710464239,  0.163935899734 );
    *   ...
    *   points[...] = vector3_type( -0.060189183801, -0.305664300919, -0.289155662060 );
    *
    *   normals[0] = vector3_type(  0.121994487941, -0.001839586534,  0.992529094219 );
    *   ...
    *   normals[...] = vector3_type( -0.021462006494,  0.442783534527, -0.896371662617 ); 
    *
    *   grid_type phi;
    *
    *   phi.create( vector3_type(-2,-2,-2), vector3_type(2,2,2), 64,64,64);
    *   detail::implicit_function<vector3_type> F = variational_interpolator(m_points.begin(),m_points.end(),m_normals.begin(),m_normals.end());
    *   for(unsigned int k=0;k<64;++k)
    *     for(unsigned int j=0;j<64;++j)
    *       for(unsigned int i=0;i<64;++i)
    *       {
    *         vector3_type coord;
    *         OpenTissue::grid::idx2coord(m_phi,i,j,k,coord);
    *         m_phi(i,j,k) = F(coord);
    *       }
    *
    * This is a bit nitty-griffty, so it may be better to use the variational-interpolator function
    * to create a implicit function functor on the fly. That is create some function like
    *
    *   template<typename grid_type,typename functor_type>
    *   void doit(grid_type & phi, functor_type & F)
    *   {
    *     typedef typename functor_type::return_type   vector3_type;  
    *     for(unsigned int k=0;k<phi.K();++k)
    *       for(unsigned int j=0;j<phi.J();++j)
    *         for(unsigned int i=0;i<phi.I();++i)
    *         {
    *           vector3_type coord;
    *           OpenTissue::grid::idx2coord(phi,i,j,k,coord);
    *           phi(i,j,k) = F(coord);
    *         }
    *    }
    *
    * and then simply write
    *
    *   typedef ....  vector3_type;
    *   typedef ...   grid_type;
    *   points[0] = vector3_type(  0.008337971754, -0.393710464239,  0.163935899734 );
    *   ...
    *   points[...] = vector3_type( -0.060189183801, -0.305664300919, -0.289155662060 );
    *   normals[0] = vector3_type(  0.121994487941, -0.001839586534,  0.992529094219 );
    *   ...
    *   normals[...] = vector3_type( -0.021462006494,  0.442783534527, -0.896371662617 ); 
    *   grid_type phi;
    *   phi.create( vector3_type(-2,-2,-2), vector3_type(2,2,2), 64,64,64);
    *   doit( phi, variational_interpolator(points.begin(),points.end(),normals.begin(),normals.end()) );
    *
    * That's it.
    *
    * @param point_begin
    * @param point_end
    * @param normal_begin
    * @param normal_end
    *
    * @return                 The resulting variational interpolation function object.
    */
    template<typename iterator>
    inline   detail::ImplicitFunction<typename iterator::value_type> variational_interpolator(
      iterator point_begin
      , iterator point_end
      , iterator normal_begin
      , iterator normal_end
      )
    {
      typedef typename iterator::value_type                 small_vector_type;//--- Hmmm, use iterator_traits for this!
      typedef detail::ImplicitFunction<small_vector_type>   implicit_function_type;
      implicit_function_type  f;
      f.init(point_begin,point_end,normal_begin,normal_end);
      return f;
    }

  } // namespace interpolation

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_VARIATIONAL_INTERPOLATOR_H
#endif
