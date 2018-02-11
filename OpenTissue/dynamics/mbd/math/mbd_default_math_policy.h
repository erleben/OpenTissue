#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_MBD_DEFAULT_UBLAS_MATH_POLICY_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_MBD_DEFAULT_UBLAS_MATH_POLICY_H
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

#include <OpenTissue/core/math/math_basic_types.h> 

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * The  Typebinder takes a template parameter: math_policy_. This
    * template parameter passes along all math types to the mbd engine.
    *
    * For convenience we have created the class default_ublas_math_policy. The
    * intention is that it can be used as the math_policy argument by users
    * that do not care about what kind of matrix libraries the  Engine
    * should rely on.
    */
    template< typename real_type_>
    class default_ublas_math_policy 
      : public OpenTissue::math::BasicMathTypes<real_type_,size_t>
    {
    public:
      typedef typename OpenTissue::math::BasicMathTypes<real_type_, size_t>   basic_math_types;
      typedef typename basic_math_types::real_type                            real_type;
      typedef typename basic_math_types::index_type                           index_type;

    public:

      typedef ublas::compressed_matrix<real_type_>    matrix_type;
      typedef ublas::vector<real_type_>               vector_type;
      typedef typename vector_type::size_type         size_type;
      typedef ublas::vector<index_type>               idx_vector_type;
      typedef matrix_type                             system_matrix_type;
      typedef ublas::vector_range< vector_type >      vector_range;
      typedef ublas::matrix_range< matrix_type >      matrix_range;
      typedef ublas::vector_range< idx_vector_type >  idx_vector_range;

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
        x.resize(n);
        x.clear();
      }

      static void resize(idx_vector_type & x,size_type n)
      {
        x.resize(n);
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
        A.resize(J.size1(), J.size1(),false);
        // Sparse matrix product is the fastest method in uBLAS for this. 
        // However, it does not work in version 1.33.1 
        //
        // It seems that sparse_prod have problems with really huge
        // matrices, above 60Kx60K we often experience an unhandled
        // exception. Which seems to come from an allocation failure (perhaps
        // a compiler operating system thing?)
        matrix_type JT;
        JT.resize(J.size2(), J.size1(), false);
        ublas::noalias(JT) = ublas::trans(J);
        matrix_type WJT;
        WJT.resize(invM.size1(), JT.size2(), false);
        ublas::noalias(WJT) = ublas::sparse_prod<matrix_type>( invM, JT);
        ublas::noalias(A) = ublas::sparse_prod<matrix_type>(J, WJT );
      }

    public:

      ///< Interface to support the GaussSeidel NCP solver.

      static real_type row_prod(system_matrix_type const & A,size_type i,vector_type const & x)
      {
        return OpenTissue::math::big::prod_row(A,x,i);
      }

      /**
      * This method is specifically introduced to support the GaussSeidel NCP solver.
      */
      static void init_system_matrix(system_matrix_type & A, vector_type const & x)
      {
        // do nothing
      }

      /**
      * This method is specifically introduced to support the GaussSeidel NCP solver.
      */
      static void update_system_matrix(system_matrix_type & A, size_type i, real_type const & dx)
      {
        // do nothing...
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MATH_MBD_DEFAULT_UBLAS_MATH_POLICY_H
#endif
