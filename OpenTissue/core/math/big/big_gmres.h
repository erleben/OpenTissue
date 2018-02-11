#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_GMRES_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_GMRES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_identity_preconditioner.h>
#include <OpenTissue/core/math/big/big_prod.h>
#include <OpenTissue/core/math/big/big_residual.h>
#include <OpenTissue/core/math/big/big_backsolve.h>

#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <boost/cast.hpp>             // needed for boost::numeric_cast

#include <vector>
#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {
      namespace detail
      {

        /**
        *  This method determines the rotation around the z-axis, which
        *  rotates the vector [a,b]^T into a vector with only a
        *  positive x-component.
        *
        * @param a   The x-component of the vector to be rorated.
        * @param b   The y-component of the vector to be rotated.
        * @param c   Upon return this argument holds the value of cos(theta), where theta is the angle the vector needs to be rotated around the z-axis.
        * @param s   Upon return this argument holds the value of sin(theta), where theta is the angle the vector needs to be rotated around the z-axis.
        */
        template<typename value_type>
        inline void get_rotation ( 
          value_type const & a
          , value_type const & b
          , value_type       & c
          , value_type       & s 
          )
        {
          using std::sqrt;
          using std::fabs;

          typedef OpenTissue::math::ValueTraits<value_type>  value_traits;

          assert( is_number(a) || !"get_rotation(): a was not a number");
          assert( is_number(b) || !"get_rotation(): b was not a number");

          //
          // Find the rotation theta that rotates [a b]^T into [ r 0]^T, that is find cos(theta) = c and sin(theta) such that
          //
          //    | r |   |  c s | |  a  | 
          //    | 0 | = | -s c | |  b  |
          //
          // In order to be a rotation we must also fulfill the constraint c^2+s^2 = 1. From the last row we get
          //
          //   0 = c*b - s*a
          //
          // This suggest we could choose c = a and and s = b, but we also need the unity constraint, so we have
          //
          //    r= sqrt{a^2+b^2}
          //         
          //    c = a/r
          //            
          //    s = b/r
          //
          //  Observe that we can make the simple rewriting
          //
          //    sqrt(a^2 + b^2 ) = sqrt(  a*a* (b*b/ a*a + 1)) = a * sqrt( (b/a)^2 + 1))
          //
          //  That means
          //
          //    c = 1 / sqrt( (b/a)^2 + 1))
          //    s = b/a * c
          //
          //  Note also that b/a = tan theta = s/c. Also note that we could have choosen to rewrite a little differently
          //
          //    sqrt(a^2 + b^2 ) = sqrt(  b*b* (a*a/ b*b + 1)) = b * sqrt( (a/b)^2 + 1))
          //
          //  That means
          //
          //    s = 1 / sqrt( (a/b)^2 + 1))
          //    c = a/b * s
          //
          //
          // (The following text is taken from: http://en.wikipedia.org/wiki/Givens_rotation
          //
          // However, to avoid overflow and underflow, we use a different computation,
          // employing the ratio b/a (which is tan theta) or its reciprocal (Golub & Van Loan 1996).
          // And as Edward Anderson (2000) discovered in improving LAPACK, a previously
          // overlooked numerical consideration is continuity. To achieve this, we required
          // r to be positive.
          //
          //if (b = 0){
          //  c = copysign(1,a);
          //  s = 0; 
          //  r = fabs(a)
          //} else if (a = 0) {
          //  c = 0;
          //  s = copysign(1,b);
          //  r = fabs(b)
          //} else if (fabs(b) > fabs(a)) {
          //  t = a/b
          //  u = copysign(sqrt(1+t*t),b)
          //  s = 1/u
          //  c = s*t
          //  r = b*u
          //} else {
          //  t = b/a
          //  u = copysign(sqrt(1+t*t),a)
          //  c = 1/u
          //  s = c*t
          //  r = a*u
          //}
          //
          // This is written in terms of the IEEE 754r copysign(x,y) function,
          // which provides a safe and cheap way to copy the sign of y to x. If
          // that is not available, x*sng(y), using the signum function, is an alternative.
          //

          if ( b == value_traits::zero() )  
          {
            c = (a>0) ? value_traits::one() : -value_traits::one(); 
            s = value_traits::zero();
          }
          else if ( a == value_traits::zero() )  
          {
            c = value_traits::zero();
            s = (b>0) ? value_traits::one() : - value_traits::one(); 
          }
          else if ( fabs( b ) > fabs( a ) )  
          {
            value_type t = a / b;
            value_type u = sqrt ( value_traits::one() + t*t );
            u = (b > value_traits::zero()) ? u : -u;
            s = value_traits::one() / u;
            c = t * s;
          }
          else //if ( abs ( a ) > abs ( b ) )  
          {
            value_type t = b / a;
            value_type u = sqrt ( value_traits::one() + t*t );
            u = (a > value_traits::zero()) ? u : -u;
            c = value_traits::one() / u;
            s = t * c;
          }

          assert( is_number(c) || !"get_rotation(): c was not a number");
          assert( is_number(s) || !"get_rotation(): s was not a number");
          assert( c <=  value_traits::one() || !"get_rotation(): illegal value of c");
          assert( c >= -value_traits::one() || !"get_rotation(): illegal value of c");
          assert( s <=  value_traits::one() || !"get_rotation(): illegal value of s");
          assert( s >= -value_traits::one() || !"get_rotation(): illegal value of s");          
          assert( fabs( (c*c+s*s) - value_traits::one() ) < math::working_precision<value_type>() || !"get_rotation(): Not a valid rotation");
        }

        /**
        *  This method applies a rotation, theta, around the z-axis, to the vector [a,b]^T.
        *
        * @param a   Upon call this argument holds the intial value of the x-component of the vector to be rorated. Upon return this argument holds the result of the rotation.
        * @param b   Upon call this argument holds the intial value of the y-component of the vector to be rorated. Upon return this argument holds the result of the rotation.
        * @param c   This argument holds the value of cos(theta), where theta is the angle the vector needs to be rotated around the z-axis.
        * @param s   This argument holds the value of sin(theta), where theta is the angle the vector needs to be rotated around the z-axis.
        */
        template<typename value_type>
        inline void set_rotation ( 
          value_type & a
          , value_type & b
          , value_type const & c
          , value_type const & s 
          )
        {
          using std::fabs;

          typedef OpenTissue::math::ValueTraits<value_type>  value_traits;

          assert( is_number(a) || !"set_rotation(): a was not a number");
          assert( is_number(b) || !"set_rotation(): b was not a number");
          assert( is_number(c) || !"set_rotation(): c was not a number");
          assert( is_number(s) || !"set_rotation(): s was not a number");
          assert( c <=  value_traits::one() || !"set_rotation(): illegal value of c");
          assert( c >= -value_traits::one() || !"set_rotation(): illegal value of c");
          assert( s <=  value_traits::one() || !"set_rotation(): illegal value of s");
          assert( s >= -value_traits::one() || !"set_rotation(): illegal value of s");
          assert( fabs( (c*c+s*s) - value_traits::one() ) < math::working_precision<value_type>() || !"set_rotation(): Not a valid rotation");
          //
          // This method computes
          //
          //  |a'|    |  c  s | | a |
          //  |b'| =  | -s  c | | b |
          //
          //
          // and returns dx' and dy' as values of the arguments dx and dy
          value_type temp  =  c * a + s * b;
          b = c * b - s * a;
          a = temp;
        }

        /**
        * Transform Hessenberg Matrix.
        *
        * This method transforms least squares subproblem into upper
        * triangular form. Below follows an outline of how this works.
        *
        * This method is invoked after the Arnoldi method has generated
        * the next vector for the span of the Krylow subspace.
        *
        * The Arnoldi method results in the factorization
        *
        *   A V_M = V_{M+1} H_{M} 
        *
        * Where H_{M} is a Hessenberg matrix of dimension (M+1 \times M). The
        * columns of V_M+1 is an orthonormal basis for the Krylow
        * subspace K_m = span{ r_0, A r_0, ..., A^{M-1} r_0}, where the
        * initial residual r_0 = b - A x_0 is given from an initial solution
        * guess x_0.
        *
        * GMRES minimizes the square of the residual norm restricted to the subspace K_m.
        *
        * So, the residual norm is given by
        *
        *     \norm{b-Ax}
        *
        * For a projection method we have x = x_0 + V_m y, so
        *
        *   b - A x = b - A(x_0 + V_m y)
        *           = r_0 - A V_M y
        *
        * Use the ``factorization'' we get from the Arnoldi Method
        *
        *  b - A x = r_0 - V_{M+1} H_{M} y
        *
        * Define beta as \norm{r_0}, note that the first column, v_1, of V_{m+1} is
        * a unit vector in the direction of r_0 (because the Arnoldi method is basically
        * a modified Graham-Schmidt ortonormalization of K_M).
        *
        *  b - A x = beta v_1 - V_{M+1} H_{M} y
        *          = V_{M+1} ( beta e_1 -  H_{M} y )
        *
        * Since the columns of V_{M+1} are orthonormal we have
        *
        *  | r | = | b - A x | = | beta e_1 -  H_{M} y |
        *
        * It can be shown that the solution for the minimizer of the square of the residual norm is given by
        *
        *   y = R_M^-1 g_m
        *
        * where R_M is an M\times M matrix obtained from transforming H_{M} into upper triangular
        * form and deleting the last row. g_m is the resulting vector from applying the same
        * transformations to the right-hand side beta e_1, and deleting the last entry.
        *
        * The transformation to upper triangular form can be done iteratively in
        * tandem with the Arnoldi method.
        *
        * The idea is to apply Givens rotations and store these. Whenever H_M is
        * expanded with a new row and column. The neat thing about this is that
        * after having transformed the system into upper triangular form the last
        * entry of the right hand side vector contains the value of the current
        * residual iterate.
        *
        * @param j   The index of the last added column to the matrix H. The new row will have index j+1.
        * @param H   The hessenberg matrix.
        * @param g   The right hand side vector.
        * @param c   A vector storing the value of cos(theta) of all the previously applied Given rotations.
        * @param s   A vector storing the value of sin(theta) of all the previously applied Given rotations.
        */
        template<typename size_type, typename matrix_type, typename vector_type>
        inline void hessenberg_matrix_transform(
          size_type j          
          , matrix_type & H
          , vector_type & g
          , vector_type & c
          , vector_type & s
          )
        {
          using std::fabs;

          typedef typename vector_type::value_type                    value_type;
          typedef          OpenTissue::math::ValueTraits<value_type>  value_traits;

          // After the j'th iteration of the Arnoldi method
          // H is grown by one row and one column. All entries
          // in the new column are non-zero, the remaining entries
          // are all zero (those in the new row but not in the
          // new column).
          //
          // First we need to apply all the previous Given rotations
          // to the new column
          //
          //
          for (size_type k = 0; k < j; ++k )
          {
            value_type a = H(   k, j );
            value_type b = H( k+1, j );
            set_rotation ( a, b, c[ k ], s[ k ] );
            H(   k, j ) = a;
            H( k+1, j ) = b;
          }

          // Next we need to find the rotation that wil transform
          // the H_{j+1,j} element into zero. 
          value_type a = H(j,j);
          value_type b = H(j+1,j);
          get_rotation ( a, b, c[j], s[j] );

          // Hereafter we apply the new rotation, because of the structure of
          // H and the rotation we only need to apply this rotation to the new row and column of H. 
          a = H(   j, j );
          b = H( j+1, j );
          set_rotation ( a, b, c[j], s[j] );

          assert( a > value_traits::zero()                        || !"hessenberg_matrix_transform(): invalid rotation, diagonal should be positive?"); 
          assert( fabs(b) < math::working_precision<value_type>() || !"hessenberg_matrix_transform(): invalid rotation, lower diagonal is nonzero?"); 

          //
          // See Proporsition 6.9 part 1 in the book of Saad (page 169 in second edition)
          //
          if( fabs(a) < math::working_precision<value_type>())
            throw std::logic_error("A matrix is singular");

          H( j  ,j ) = a;
          H( j+1,j ) = b;

          // Finally we can update the right hand side vector with the same rotation.
          set_rotation ( g[ j ], g[ j+1 ], c[ j ], s[ j ] );
        }

        /**
        *  Test if basis is orthonormal.
        *  This method is only used for internal testing purpose. 
        *
        * @param m         The number of vectors in the basis for the preconditioned Krylov
        *                  subspace: K_m = span{r_0, M^{-1} A r_0,...,( M^{-1} A )^{m-1} r_0, } . 
        *                  That is the indices of the basis vectors range from v_0 to v_{m-1}.
        * @param v         The vectors for the basis of the Krylow subspace.
        */
        template<typename size_type, typename vector_type>
        inline bool is_orthonormal ( 
          size_type m
          , std::vector<vector_type> const & v 
          )
        {
          using std::fabs;

          typedef typename vector_type::value_type                    value_type;
          typedef          OpenTissue::math::ValueTraits<value_type>  value_traits;

          assert( m>0         || !"is_orthonormal(): m was out of range");
          assert( m<=v.size() || !"is_orthonormal(): m was out of range");

          value_type const precision = ::boost::numeric_cast<value_type>(10e-6);

          for ( size_type i = 0; i < m; ++i )
          {
            value_type tmp = inner_prod( v[i], v[i] );
            if( fabs(tmp-value_traits::one()) > precision )
              return false;
          }
          for ( size_type i = 0; i < m; ++i )
            for ( size_type j = i+1; j < m; ++j )
            {
              value_type tmp = inner_prod( v[i], v[j] );
              if( fabs(tmp) >  precision )
                return false;

            }
            return true;
        }

        /**
        *  Test if matrix is upper triangular.
        *  This method is only used for internal testing purpose. 
        *
        * @param m         The number of vectors in the basis for the preconditioned Krylov
        *                  subspace: K_m = span{r_0, M^{-1} A r_0,...,( M^{-1} A )^{m-1} r_0, } . 
        *                  That is the indices of the basis vectors range from v_0 to v_{m-1}.
        * @param H         The matrix that should be tested.
        */
        template<typename size_type, typename matrix_type>
        inline bool is_upper_triangular ( 
          size_type m
          , matrix_type const & H
          )
        {
          typedef typename matrix_type::value_type                    value_type;
          typedef          OpenTissue::math::ValueTraits<value_type>  value_traits;

          assert( m>0              || !"is_upper_triangular(): m was out of range");
          assert( H.size1() >= m   || !"is_upper_triangular(): size of H was incompatible");
          assert( H.size2() >= m-1 || !"is_upper_triangular(): size of H was incompatible");

          for ( size_type i = 0; i < m; ++i )
            for ( size_type j = i+1; j < m; ++j )
            {
              value_type h_ij = H(i,j);
              if(i>j && h_ij > value_traits::zero())
                return false;
            }
            return true;
        }

        /**
        *  Update the current iterate.
        *  This method computes 
        *
        *    x = x_0 + V_M y
        *
        *  where 
        *
        *    y = arg min \norm { b - A x}
        *            y
        *
        *
        * @param x         The current value of the iterate (intially x_0). Upon
        *                  return this argument holds the updated value of the
        *                  iterate x.
        * @param m         The number of vectors in the basis for the preconditioned Krylov
        *                  subspace: K_m = span{r_0, M^{-1} A r_0,...,( M^{-1} A )^{m-1} r_0, } . 
        *                  That is the indices of the basis vectors range from v_0 to v_{m-1}.
        * @param H         The Hessenberg matrix transformed into triangular form. The triangular
        *                  form consist of the m \times m submatrix of H.
        * @param g         The right hand side vector for the least square sub problem
        *                  transformed into triangular form. The transformed vector is the m-dimensional
        *                  subpart of g.
        * @param v         The vectors for the basis of the Krylow subspace.
        */
        template<typename size_type, typename vector_type, typename matrix_type>
        inline void update ( 
          vector_type & x
          , size_type m
          , matrix_type & H
          , vector_type const & g
          , std::vector<vector_type> const & v 
          )
        {
          assert( m>0                         || !"update(): m was out of range");
          assert( m<=g.size()                 || !"update(): m was out of range");
          assert( is_orthonormal( m, v )      || !"update(): basis where not orthonormal?");
          assert( is_upper_triangular( m, H ) || !"update(): H where not upper triangular?");

          vector_type y;       
          OpenTissue::math::big::backsolve( m, H, y, g);

          for ( size_type j = 0; j < m; ++j )
            x += y[j]*v[j];
        }

      }// namespace detail

      /**
      *  GMRES  Generalized Minimum Residual Method.
      * GeneralizedMinimalResidualSolver solves the non-symmetric linear 
      * system Ax = b using the Generalized Minimal Residual method
      *  See:
      *      http://en.wikipedia.org/wiki/GMRES
      *      http://netlib2.cs.utk.edu/linalg/html_templates/report.html
      *      Saad's Iterative Methods for Sparse Linear Systems
      *      http://www-users.cs.umn.edu/~saad/books.html
      *
      * We will try to use the same default values as the GMRES method implemented in Matlab. If a value is zero then it is
      * assumed that the parameter were left unspecified by end-user.
      *  
      *
      * From the Matlab help:
      * 
      *    X = GMRES(A,B) attempts to solve the system of linear equations A*X = B for
      *    X.  The N-by-N coefficient matrix A must be square and the right hand side
      *    column vector B must have length N.  A may be a function returning A*X. This
      *    uses the unrestarted method with MIN(N,10) total iterations.
      *
      *    GMRES(A,B,RESTART) restarts the method every RESTART iterations.  If RESTART
      *    is N or [] then GMRES uses the unrestarted method as above.
      *
      *    GMRES(A,B,RESTART,TOL) specifies the tolerance of the method.  If TOL is []
      *    then GMRES uses the default, 1e-6.
      * 
      *    GMRES(A,B,RESTART,TOL,MAXIT) specifies the maximum number of outer
      *    iterations. Note: the total number of iterations is RESTART*MAXIT. If MAXIT
      *    is [] then GMRES uses the default, MIN(N/RESTART,10). If RESTART is N or []
      *    then the total number of iterations is MAXIT.
      *
      * @param A    The system matrix.
      * @param x    Upon return this argument holds the solution vector.
      * @param b   The right hand side vector.
      * @param max_itreations           The maximum number of outer iterations that can be used.
      * @param max_restart_iterations   The restarting parameter for GMRES (see Saad's Iterative Methods for Sparse Linear Systems).
      * @param tolerence                      The threshold used in stop criteria
      * @param relative_residual_error  Upon return this argument holds the accuracy of the last solution computed by invoking the solve methods.
      * @param used_inner_iterations    Upon return this argument holds the number of used inner iterations.
      * @param used_outer_iterations    Upon return this argument holds the number of used outer iterations.
      * @param status    Upon return this argument holds the status of the GMRES method.
      *  status flag of GMRES,
      *   0 : GMRES converged to the desired tolerance within maximum iterations.
      *   1 : GMRES iterated maximum iterations but did not converge
      *   2 : Preconditioner was ill-conditioned
      *   3 : GMRES stagnated (two consecutive iterates were the same)
      * @param P                  A preconditioner.
      */
      template<typename matrix_type, typename vector_type, typename preconditioner_type>
      inline void gmres(
        matrix_type const & A
        , vector_type & x
        , vector_type const & b 
        , typename vector_type::size_type const & max_iterations         
        , typename vector_type::size_type const & max_restart_iterations 
        , typename vector_type::value_type const & tolerance                   
        , typename vector_type::value_type & relative_residual_error
        , typename vector_type::size_type & used_inner_iterations
        , typename vector_type::size_type & used_outer_iterations     
        , typename vector_type::size_type & status
        , preconditioner_type const & P
        )
      {
        using std::fabs;
        using std::min;
        using std::ceil;

        typedef typename matrix_type::value_type                     value_type;
        typedef typename vector_type::size_type                      size_type;
        typedef          OpenTissue::math::ValueTraits<value_type>   value_traits;

        static size_type const ten = ::boost::numeric_cast<size_type>(10);

        size_type  const & N       = b.size();
        size_type  const & R       = max_restart_iterations;
        size_type  const & M       = max_iterations;
        value_type const & eps     = tolerance>0 ? tolerance : boost::numeric_cast<value_type>( 1e-6 );
        bool       const restarted = R!=N && R>0;
        size_type        maxit     = M;

        if(maxit==0)
        {
          if (restarted)
          {
            size_type  const fraction  = ::boost::numeric_cast<size_type>( ceil( value_traits::one()*N / R ) );

            maxit = min( fraction, ten);
          }
          else
          {
            maxit = min( N, ten);
          }
        }

        size_type  const outer     = restarted ? maxit : 1;
        size_type  const inner     = restarted ? R     : maxit;

        used_inner_iterations = 0;
        used_outer_iterations = 0;
        relative_residual_error = value_traits::infinity();

        value_type  norm_b = ublas::norm_2 ( b );

        // Check for all zero right hand size vector => all zero solution
        if(norm_b < eps )
        {
          // If rhs vector is all zeros then solution is
          // all zeros. Thus a valid solution have been
          // obtained the relative residual is actually 0/0,
          // no iterations need be performed.
          status = 0;
          x.clear();
          relative_residual_error = value_traits::zero();
          return;
        }

        vector_type r( N );                // Allocate space for the residual vector.

        value_type  const rel_eps  = eps*norm_b; // Relative tolerance

        // Compute the residual vector
        residual(A, x, b, r);
        value_type norm_r = ublas::norm_2 ( r );

        // Check if we have a good approximation already
        if ( norm_r <= rel_eps )
        {
          status = 0;
          relative_residual_error = norm_r / norm_b;
          return;
        }

        value_type  norm_r_min = norm_r;    // The value of the minimum residual norm.
        vector_type x_min      = x;         // Iterate which has minimal residual so far.
        size_type   k_min      = 0;         // "Outer" iteration at which xmin was computed.
        size_type   j_min      = 0;         // "Inner" iteration at which xmin was computed.

        status     = 1;        

        // create data container for keeping vectors for the basis of the preconditioned Krylov subspace K_M
        std::vector<vector_type> v; 
        v.resize(inner + 1);       

        // Allocate space for all other temporary data 
        vector_type tmp( N );               // Allocate space for temporary vector.
        matrix_type H( inner+1, inner );    // Get space for Hessenberg matrix
        vector_type g( inner+1 );           // Allocate space for the right hand side vector of the least squares sub problem.
        vector_type c( inner+1 );           // Allocate space for storing the values of cos(theta) of the Givens rotations.
        vector_type s( inner+1 );           // Allocate space for storing the values of sin(theta) of the Givens rotations.
        vector_type w( N );                 // Allocate space for vector used to hold vectors generated Arnoldi method.

        // To save storage and computations we use a restart technique.
        for(size_type k=1; k<=outer; ++k )
        {
          // Use preconditioner to get preconditioned residual
          tmp.clear();
          P( A, tmp, r );
          r = tmp;

          // TODO: At this point we should see if the pre-conditioner was ill-conditioned => m_status = 2

          // Perform the Arnoldi Method to find a basis for the preconditioned Krylov subspace.
          value_type beta = ublas::norm_2 ( r );
          v[0].resize ( N );
          v[0] = r / beta;

          g.clear();
          g[0] = beta;

          size_type used_inner = 0;  // Auxiliary variable used to keep track of how many loops we did = #number of vectors in Krylov subspace

          for (size_type j = 0; j < inner; ++j ) 
          {
            ++used_inner;

            // compute the next vector, w_j = M^{-1} A v_j, of the preconditioned Krylov subspace
            prod( A, v[j], tmp);
            w.clear();
            P( A, w, tmp);

            // TODO: At this point we should see if the pre-conditioner was ill-conditioned, => m_status = 2

            // Perform the orthonomalization of the Arnoldi Method. This
            // is a modified Gram-Schmidt ortonormalization.
            for (size_type  i = 0; i <= j; ++i )
            {
              value_type h_ij = ublas::inner_prod ( w, v[i] );
              w -= ( h_ij * v[i] );
              H( i, j ) = h_ij;
            }
            value_type h_jp1j = ublas::norm_2( w );
            v[j+1].resize(N);
            v[j+1]   = w/h_jp1j; 
            H(j+1,j) = h_jp1j;
            //
            // kenny: if H(j+1,j) is zero then we should make an early exit
            // with m=j, see line 8 in Algorithm 6.9 in the book by Saad (page
            // 169 in second edition).
            //
            // This is not a problem! The Givens rotation
            //
            // | r|   |  c s | | a |
            // | 0| = | -s c | | b |
            //
            // For the case r>0, a!=0 and b=0, is c=sign(a) 1 and s=0. If we apply
            // this Givens rotation to the current right hand side vector
            //
            //  g_j != 0 and g_j+1==0
            //
            // Then we see that the value of the transformed g_j+1 value is zero! This
            // means the residual error drops to zero and the algorithm will make an early
            // exit when during the convergence testing.
            //
            // However, making the explicit test for H(j+1,j) is zero would allow
            // us to skip the Hessenberg matrix transformation by Givens Rotations
            // in this particular case. This is not implemented!


            // Apply Givens rotations
            // see http://en.wikipedia.org/wiki/GMRES
            detail::hessenberg_matrix_transform( j, H, g, c, s);

            // TODO: Hm, roughly about here we should test for stagnation? => status = 3


            // From theory we know that the last entry of the right hand side vector
            // of the upper triangular sub-problem is equal to the norm of the
            // residual. Thus g[i+1] = \norm{b - A x}
            norm_r = fabs( g[j+1] );

            if(norm_r < norm_r_min)
            {
              norm_r_min = norm_r;
              k_min = k;
              j_min = used_inner;
              //x_min = ? Huba, I am not sure what would be the best way to get the value of this iterate?
            }
            if( norm_r  < rel_eps )
            {
              status = 0;
              used_inner_iterations = used_inner;
              used_outer_iterations = k ;
              break;
            }
          }

          // Update iterate x
          //
          // At this point we have generated the vectors 
          //
          //  v_0, ..., v_{j+1}
          //
          // So the total number of generated vectors are |j+2| vectors.
          // The vector v_{j+1} is not needed, so we have |j+1| vectors
          // in the basis for the Krylov subspace. 
          //
          detail::update ( x, used_inner, H, g, v );

          // Check for convergence
          residual(A, x, b, r);           
          norm_r = ublas::norm_2 ( r );

          if(norm_r < norm_r_min)
          {
            norm_r_min = norm_r;
            k_min = k;
            j_min = used_inner;
            x_min = x;
          }
          if ( norm_r < rel_eps )
          {
            status = 0;
            used_inner_iterations = used_inner;
            used_outer_iterations = k;
            break;
          }
        }
        if ( status == 0 )
        {
          // GMRES did converge
          relative_residual_error = norm_r / norm_b;
        }
        else
        {
          // GMRES did not converge, or preconditioner was ill-conditioned, or iterate was stagnated!!!
          x = x_min;
          used_inner_iterations = j_min;
          used_outer_iterations = k_min;
          relative_residual_error = norm_r_min / norm_b;
        }
      }

      template<typename matrix_type, typename vector_type>
      inline void gmres(
        matrix_type const & A
        , vector_type & x
        , vector_type const & b 
        , typename vector_type::size_type const & max_iterations         
        , typename vector_type::size_type const & max_restart_iterations 
        , typename vector_type::value_type const & tolerance                   
        , typename vector_type::value_type & relative_residual_error
        , typename vector_type::size_type & used_inner_iterations
        , typename vector_type::size_type & used_outer_iterations     
        , typename vector_type::size_type & status
        )
      {
        IdentityPreconditioner preconditioner;

        gmres( A,x,b,max_iterations,max_restart_iterations,tolerance, relative_residual_error, used_inner_iterations, used_outer_iterations, status, preconditioner);
      }

      template<typename matrix_type, typename vector_type>
      inline void gmres(
        matrix_type const & A
        , vector_type & x
        , vector_type const & b 
        , typename vector_type::size_type const & max_iterations         
        , typename vector_type::size_type const & max_restart_iterations 
        , typename vector_type::value_type const & tolerance                   
        )
      {
        typename vector_type::value_type relative_residual_error;
        typename vector_type::size_type used_inner_iterations;
        typename vector_type::size_type used_outer_iterations;    
        typename vector_type::size_type status;
        
        IdentityPreconditioner preconditioner;
        
        gmres( A,x,b,max_iterations,max_restart_iterations,tolerance, relative_residual_error, used_inner_iterations, used_outer_iterations, status, preconditioner);
      }


      template<typename matrix_type, typename vector_type>
      inline void gmres(
        matrix_type const & A
        , vector_type & x
        , vector_type const & b 
        )
      {
        typedef OpenTissue::math::ValueTraits< typename vector_type::value_type> value_traits;

        typename vector_type::size_type  max_iterations          = 0; 
        typename vector_type::size_type  max_restart_iterations  = 0;
        typename vector_type::value_type tolerance               = value_traits::zero();
        typename vector_type::value_type relative_residual_error = value_traits::zero();
        typename vector_type::size_type  used_inner_iterations   = 0;
        typename vector_type::size_type  used_outer_iterations   = 0;
        typename vector_type::size_type  status                  = 0;

        IdentityPreconditioner   preconditioner;

        gmres( A,x,b,max_iterations,max_restart_iterations,tolerance, relative_residual_error, used_inner_iterations, used_outer_iterations, status, preconditioner);
      }


      /**
      * GMRES Functor.
      * This is a convenience class providing
      * a functor interface for all the free template functions.
      *
      * It is the intention that this will make it easier for
      * end-users to pass the GMRES method to other algorithms.
      */
      class GMRESFunctor
      {
      public:

        template<typename matrix_type, typename vector_type, typename preconditioner_type>
        void operator()(
          matrix_type const & A
          , vector_type & x
          , vector_type const & b 
          , typename vector_type::size_type const & max_iterations         
          , typename vector_type::size_type const & max_restart_iterations 
          , typename vector_type::value_type const & tolerance                   
          , typename vector_type::value_type & relative_residual_error
          , typename vector_type::size_type & used_inner_iterations
          , typename vector_type::size_type & used_outer_iterations     
          , typename vector_type::size_type & status
          , preconditioner_type const & P
          ) const
        {
          gmres(A,x,b,max_iterations,max_restart_iterations,tolerance,relative_residual_error,used_inner_iterations,used_outer_iterations,status,P);
        }

        template<typename matrix_type, typename vector_type>
        void operator()(
          matrix_type const & A
          , vector_type & x
          , vector_type const & b 
          , typename vector_type::size_type const & max_iterations         
          , typename vector_type::size_type const & max_restart_iterations 
          , typename vector_type::value_type const & tolerance                   
          , typename vector_type::value_type & relative_residual_error
          , typename vector_type::size_type & used_inner_iterations
          , typename vector_type::size_type & used_outer_iterations     
          , typename vector_type::size_type & status
          ) const
        {
          gmres(A,x,b,max_iterations,max_restart_iterations,tolerance,relative_residual_error,used_inner_iterations,used_outer_iterations,status);
        }

        template<typename matrix_type, typename vector_type>
        void operator()(
          matrix_type const & A
          , vector_type & x
          , vector_type const & b 
          , typename vector_type::size_type const & max_iterations         
          , typename vector_type::size_type const & max_restart_iterations 
          , typename vector_type::value_type const & tolerance                   
          ) const
        {
          gmres(A,x,b,max_iterations,max_restart_iterations,tolerance);
        }

        template<typename matrix_type, typename vector_type>
        void operator()(
          matrix_type const & A
          , vector_type & x
          , vector_type const & b 
          ) const
        {
          gmres(A,x,b);
        }
      };

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_GMRES_H
#endif
