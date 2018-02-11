#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_MBD_OPTIMIZED_UBLAS_MATH_POLICY_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_MBD_OPTIMIZED_UBLAS_MATH_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h> 
#include <OpenTissue/core/math/big/big_prod.h>
#include <OpenTissue/core/math/big/big_prod_row.h>
#include <OpenTissue/core/math/big/big_prod_add_rhs.h>
#include <OpenTissue/core/math/big/big_prod_sub_rhs.h>
#include <OpenTissue/core/math/big/big_prod_sub.h>
#include <OpenTissue/core/math/big/big_prod_add.h>
#include <OpenTissue/core/math/big/big_prod_trans.h>

#include <OpenTissue/core/math/big/io/big_matlab_write.h> 

#include <OpenTissue/dynamics/mbd/math/mbd_math_update_f.h>
#include <OpenTissue/dynamics/mbd/math/mbd_math_compute_f.h>
#include <OpenTissue/dynamics/mbd/math/mbd_math_compute_diagonal.h>
#include <OpenTissue/dynamics/mbd/math/mbd_math_compute_WJT.h>

#include <OpenTissue/core/math/math_basic_types.h> 

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * The  Typebinder takes a template parameter: math_policy_. This
    * template parameter passes along all math types to the mbd engine.
    *
    * This class implements an optimized math policy using boost uBLAS and
    * highly specialized for the non-linear projected Gauss-Seidel solver.
    */
    template< typename real_type_>
    class optimized_ublas_math_policy 
      : public OpenTissue::math::BasicMathTypes<real_type_,size_t>
    {
    public:
      typedef typename OpenTissue::math::BasicMathTypes<real_type_,size_t>   basic_math_types;
      typedef typename basic_math_types::index_type                          index_type;
      typedef typename basic_math_types::real_type                           real_type;

    public:

      typedef ublas::compressed_matrix<real_type_>    matrix_type;
      typedef ublas::vector<real_type_>               vector_type;
      typedef typename vector_type::size_type         size_type;
      typedef ublas::vector<index_type>               idx_vector_type;
      typedef ublas::vector_range< vector_type >      vector_range;
      typedef ublas::matrix_range< matrix_type >      matrix_range;
      typedef ublas::vector_range< idx_vector_type >  idx_vector_range;

    public:

      class system_matrix_type
      {
      public:

        matrix_type m_J;
        matrix_type m_WJT;
        vector_type m_f;
        vector_type m_d;

        real_type operator() (size_type const & i, size_type const & j)const 
        {
          assert(i==j || !"system_matrix_type::operator(i,j): i was not equal to j");
          assert(i<m_d.size() || !"system_matrix_type::operator(i,j): i out of bound");
          return m_d(i);
        }

      };

    public:


      static matrix_range subrange( 
        matrix_type & M
        , size_type start1
        , size_type stop1
        , size_type start2
        , size_type stop2 
        )
      {
        return ublas::subrange(M,start1,stop1,start2,stop2);
      }

      static vector_range subrange(vector_type & v, size_type start, size_type stop )
      {
        return ublas::subrange(v,start,stop);
      }

      static idx_vector_range subrange(idx_vector_type & v, size_type start, size_type stop )
      {
        return ublas::subrange(v,start,stop);
      }

      static void get_dimension(vector_type const & v, size_type & n)
      {
        n = v.size();
      }

      static void get_dimensions(matrix_type const & A,size_type & m, size_type & n)
      {
        m = A.size1();
        n = A.size2();
      }

      static void resize(vector_type & x,size_type n)
      {
        x.resize(n,false);
        x.clear();
      }

      static void resize(idx_vector_type & x,size_type n)
      {
        x.resize(n,false);
        x.clear();
      }

      static void resize(matrix_type & A,size_type m,size_type n)
      {
        A.resize(m,n,false);
        A.clear();
      }    

      /**
      *computes: y = prod(A,x) + b
      */
      static void prod(matrix_type const & A,vector_type const & x, vector_type const & b, vector_type & y)
      {
        y.resize(A.size1(),false);        
        OpenTissue::math::big::prod_add_rhs(A,x,b,y);
      }

      /**
      *computes: y = prod(A,x) - b
      */
      static void prod_minus(matrix_type const & A,vector_type const & x, vector_type const & b, vector_type & y)
      {
        y.resize(A.size1(),false);        
        OpenTissue::math::big::prod_sub_rhs(A,x,b,y);
      }

      /**
      *computes: y = prod(A,x)
      */
      static void prod(matrix_type const & A,vector_type const & x, vector_type & y)
      {
        y.resize(A.size1(),false);        
        OpenTissue::math::big::prod(A,x,y);
      }

      /**
      *computes: y += prod(A,x)
      */
      static void prod_add(matrix_type const & A,vector_type const & x, vector_type & y)
      {
        y.resize(A.size1(),false);        
        OpenTissue::math::big::prod_add(A,x,y);
      }

      /**
      *computes: y = prod(A,x)*s
      */
      static void prod(matrix_type const & A,vector_type const & x, vector_type & y, real_type const & s)
      {
        y.resize(A.size1(),false);        
        OpenTissue::math::big::prod(A,x,s,y);
      }

      /**
      *computes: y += prod(A,x)*s
      */
      static void prod_add(matrix_type const & A,vector_type const & x, vector_type & y, real_type const & s)
      {
        y.resize(A.size1(),false);        
        OpenTissue::math::big::prod_add(A,x,s,y);
      }

      /**
      *computes: y = prod(trans(A),x)
      */
      static void prod_trans(matrix_type const & A,vector_type const & x, vector_type & y)
      {
        y.resize(A.size2(),false);        
        OpenTissue::math::big::prod_trans(A,x,y);
      }

      /**
      *computes: y = prod(trans(A),x) + b
      */
      static void prod_trans(matrix_type const & A,vector_type const & x, vector_type const & b, vector_type & y)
      {
        y.resize(A.size2(),false);        
        OpenTissue::math::big::prod_trans(A,x,b,y);
      }

      /**
      *computes: x *= s
      */
      static void prod(vector_type & x, real_type const & s )
      {
        x *= s;
      }

      /**
      *computes: x = -y
      */
      static void assign_minus(vector_type const & y,  vector_type & x)
      {
        x = -y;
      }

      static void compute_system_matrix(matrix_type const & invM, matrix_type const & J, system_matrix_type & A)
      {
        A.m_J.resize(J.size1(),J.size2(),false);
        A.m_WJT.resize(J.size1(),J.size2(),false);
        A.m_f.resize(invM.size1(),false);
        A.m_f.clear();
        A.m_d.resize(J.size1(),false);

        A.m_J.assign(J);

        math::compute_WJT(invM,J,A.m_WJT);
        math::compute_diagonal(J,A.m_WJT,A.m_d);
      }


    public:

      ///< Interface to support the GaussSeidel NCP solver.

      static real_type row_prod(system_matrix_type const & A,size_type i,vector_type const & x)
      {
        return OpenTissue::math::big::prod_row(A.m_J,A.m_f,i);
      }

      static void prod(system_matrix_type const & A,vector_type const & x, vector_type const & b, vector_type & y)
      {
        y.resize(A.m_WJT.size1(),false);
        //--- This method assumes that f is up todate and have
        //--- been computed as f = prod( prod(invM, trans(J) ), x)
        OpenTissue::math::big::prod_add_rhs(A.m_J,  A.m_f, b, y);
      }

      /**
      * This method is specifically introduced to support the GaussSeidel NCP solver.
      */
      static void init_system_matrix(system_matrix_type & A, vector_type const & x)
      {
        math::compute_f(A.m_WJT,x,A.m_f);
      }

      /**
      * This method is specifically introduced to support the GaussSeidel NCP solver.
      */
      static void update_system_matrix(system_matrix_type & A, size_type i, real_type const & dx)
      {
        math::update_f(A.m_WJT,i,dx,A.m_f);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_MBD_OPTIMIZED_UBLAS_MATH_POLICY_H
#endif
