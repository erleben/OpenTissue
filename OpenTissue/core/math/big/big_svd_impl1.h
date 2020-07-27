#ifndef OPENTISSUE_CORE_MATH_BIG_SVD_IMPL1_H
#define OPENTISSUE_CORE_MATH_BIG_SVD_IMPL1_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2009 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>  
#include <OpenTissue/core/math/math_value_traits.h>  

#include <boost/cast.hpp>             // needed for boost::numeric_cast
#include <cmath>                      // needed for std::fabs
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
				 * Hypotenuse.
				 *
				 * @param a   A real value
				 * @param b   A real vlaue
				 *
				 * @returns hypotenuse of real (non-complex) scalars a and b by avoiding underflow/overflow using (a * sqrt( 1 + (b/a) * (b/a))), rather than sqrt(a*a + b*b).
				 */
				template <class T>
				inline T hypot(T const &a, T const &b)
				{
					using std::fabs;
					using std::sqrt;
					
					T const zero = boost::numeric_cast<T>( 0.0 );
					T const one  = boost::numeric_cast<T>( 1.0 );
					
					if (a == zero)
						return fabs(b);
					else
					{
						T const c = b/a;
						
						return fabs(a) * sqrt(one + c*c);
					}
				}
				
				/**
				 * Compute Singular Value Decomposition of a matrix.
				 *
				 *     A = U S VT
				 *
				 * For an m-by-n matrix A with m >= n, the singular value decomposition is
				 * an m-by-n orthogonal matrix U, an n-by-n diagonal matrix S, and
				 * an n-by-n orthogonal matrix V so that A = U*S*V'.
                 *
				 * The singular values, sigma(k) = S(k,k), are ordered so that sigma(0) >= sigma(1) >= ... >= sigma(n-1).
                 *
				 * The singular value decompostion always exists, so the constructor will
				 * never fail.  The matrix condition number and the effective numerical
				 * rank can be computed from this decomposition.
				 *
				 * (Adapted from JAMA, a Java Matrix Library, developed by jointly by the Mathworks and NIST; see  http://math.nist.gov/javanumerics/jama).
				 *
				 * @param AE         The input matrix.
				 * @param U          Upon return this matrix holds orthogonal columns.
				 * @param s          A vector containing the singular values.
				 * @param V          Upon return this matrix is an orthogonal matrix.
				 *
				 */
				template<typename ME>
				inline void svd_impl1( 
									    ublas::matrix_expression<ME> const & AE
									  , ublas::matrix<typename ME::value_type> & U
									  , ublas::vector<typename ME::value_type> & s
									  , ublas::matrix<typename ME::value_type> & V
									  )
				{
					using std::fabs;
					using std::max;
					using std::min;
					using std::pow;
					using std::sqrt;
					
					typedef typename ME::value_type                        value_type;
					typedef ublas::matrix<value_type>                      matrix_type;
					typedef ublas::vector<typename ME::value_type>         vector_type;
					
					value_type const zero = boost::numeric_cast< value_type >( 0.0 );
					value_type const one  = boost::numeric_cast< value_type >( 1.0 );
					value_type const two  = boost::numeric_cast< value_type >( 2.0 );
					value_type const eps  = boost::numeric_cast<value_type>( pow( 2.0, -52.0 ) );
					
					//--- SVD decomposition  : A = U S VT : Dimensions: mxn =  mxn  nxn   nxn
					int const m = static_cast<int>( AE().size1() );
					int const n = static_cast<int>( AE().size2() );
					
					if(m<1)
						throw std::invalid_argument("svd(): A did not have any rows?");
					if(n<1)
						throw std::invalid_argument("svd(): A did not have any columns?");
					
					matrix_type A;
					vector_type e;
					vector_type work;
					
					int const nu = min(m,n);
					
					e.resize(n);
					work.resize( m );
					A.resize( m, n );
					U.resize(  m, nu );
					s.resize(  min(m+1,n) );
					V.resize( n,n);
					
					A = AE();
					U.clear();
					s.clear();
					V.clear();
					e.clear();
					work.clear();
					
					int const wantu = 1;  					/* boolean */
					int const wantv = 1;  					/* boolean */
					
					// Reduce A to bidiagonal form, storing the diagonal elements
					// in s and the super-diagonal elements in e.
					
					int const nct = min( m-1, n);
					int const nrt = max( 0, min( n-2, m ) );
					
					for (int k = 0; k < max(nct,nrt); ++k) 
					{
						if (k < nct) 
						{
							
							// Compute the transformation for the k-th column and
							// place the k-th diagonal in s(k).
							// Compute 2-norm of k-th column without under/overflow.
							s(k) = zero;
							for (int i = k; i < m; ++i) 
							{
								s(k) = detail::hypot(  s(k), A(i,k) );
							}
							
							if (s(k) != zero) 
							{
								if (A(k,k) < zero)
								{
									s(k) = -s(k);
								}
								for (int i = k; i < m; ++i) 
								{
									A(i,k) /= s(k);
								}
								A(k,k) += one;
							}
							
							s(k) = -s(k);
						}
						
						for (int j = k+1; j < n; ++j) 
						{
							if ( (k < nct) && (s(k) != zero) )  
							{
								// Apply the transformation.
								value_type t = zero;
								
								for (int i = k; i < m; ++i)
								{
									t += A(i,k) * A(i,j);
								}			
								
								t = - t / A(k,k);
								
								for (int i = k; i < m; ++i)
								{
									A(i,j) += t * A(i,k);
								}
							}
							
							// Place the k-th row of A into e for the
							// subsequent calculation of the row transformation.
							e(j) = A(k,j);            
						}
						
						
						if ( wantu & (k < nct) )
						{
							// Place the transformation in U for subsequent back
							// multiplication.
							for (int i = k; i < m; ++i)
							{
								U(i,k) = A(i,k);
							}
						}
						
						
						if (k < nrt)
						{
							// Compute the k-th row transformation and place the
							// k-th super-diagonal in e(k).
							// Compute 2-norm without under/overflow.
							
							e(k) = zero;
							
							for (int i = k+1; i < n; ++i)
							{
								e(k) = detail::hypot( e(k), e(i) );
							}
							
							if ( e(k) != zero ) 
							{
								if ( e(k+1) < zero ) 
								{
									e(k) = -e(k);
								}
								
								for (int i = k+1; i < n; ++i)
								{
									e(i) /= e(k);
								}
								
								e(k+1) += one;
							}
							
							e(k) = -e(k);
							
							if ( (k+1 < m) & (e(k) != zero)) 
							{
								// Apply the transformation.
								
								for (int i = k+1; i < m; ++i)
								{
									work(i) = zero;
								}
								
								for (int j = k+1; j < n; ++j)
								{
									for (int i = k+1; i < m; ++i)
									{
										work(i) += e(j) * A(i,j);
									}
								}
								
								for (int j = k+1; j < n; ++j)
								{
									value_type  t = - e(j) / e(k+1);
									for (int i = k+1; i < m; ++i)
									{
										A(i,j) += t * work(i);
									}
								}
							}
							if (wantv) 
							{
								// Place the transformation in V for subsequent
								// back multiplication.
								
								for (int i = k+1; i < n; ++i)
								{
									V(i,k) = e(i);
								}
							}
						}
					}
					
					// Set up the final bidiagonal matrix or order p.
					
					int p = min( n, m+1 );
					
					if (nct < n) 
					{
						s(nct) = A(nct,nct);
					}
					
					if (m < p) 
					{
						s(p-1) = zero;
					}
					
					if (nrt+1 < p)
					{
						e(nrt) = A(nrt,p-1);
					}
					
					e(p-1) = zero;
					
					// If required, generate U.
					
					if (wantu) 
					{
						for (int j = nct; j < nu; ++j) 
						{
							for (int i = 0; i < m; ++i) 
							{
								U(i,j) = zero;
							}
							U(j,j) = one;
						}
						
						for (int k = nct-1; k >= 0; --k)
						{
							if (s(k) != zero)
							{
								for (int j = k+1; j < nu; ++j)
								{
									value_type t = zero;
									
									for (int i = k; i < m; ++i)
									{
										t += U(i,k) * U(i,j);
									}
									
									t = -t / U(k,k);
									
									for (int i = k; i < m; ++i)
									{
										U(i,j) += t * U(i,k);
									}
								}
								
								for (int i = k; i < m; ++i ) 
								{
									U(i,k) = -U(i,k);
								}
								
								U(k,k) = one + U(k,k);
								
								for (int i = 0; i < k-1; ++i)
								{
									U(i,k) = zero;
								}
								
							}
							else
							{
								for (int i = 0; i < m; ++i)
								{
									U(i,k) = zero;
								}
								U(k,k) = one;
							}
						}
					}
					
					// If required, generate V.
					
					if (wantv) 
					{
						for (int k = n-1; k >= 0; --k) 
						{
							if ((k < nrt) & (e(k) != zero)) 
							{
								for (int j = k+1; j < nu; ++j) 
								{
									value_type t = zero;
									
									for (int i = k+1; i < n; ++i) 
									{
										t += V(i,k) * V(i,j);
									}
									
									t = - t / V(k+1,k);
									
									for (int i = k+1; i < n; ++i) 
									{
										V(i,j) += t * V(i,k);
									}
								}
							}
							
							for (int i = 0; i < n; ++i)
							{
								V(i,k) = zero;
							}
							V(k,k) = one;
						}
					}
					
					// Main iteration loop for the singular values.					
					int const pp        = p - 1;				
					int       iteration = 0;
					
					while (p > 0) 
					{
						int k    = 0;
						int kase = 0;
						
						// Here is where a test for too many iterations would go.
						
						// This section of the program inspects for
						// negligible elements in the s and e arrays.  On
						// completion the variables kase and k are set as follows.
						
						// kase = 1     if s(p) and e(k-1) are negligible and k<p
						// kase = 2     if s(k) is negligible and k<p
						// kase = 3     if e(k-1) is negligible, k<p, and
						//              s(k), ..., s(p) are not negligible (qr step).
						// kase = 4     if e(p-1) is negligible (convergence).
						
						for (k = p-2; k >= -1; --k) 
						{
							if (k == -1) 
							{
								break;
							}
							if ( fabs( e(k) ) <= eps * ( fabs(s(k) ) + fabs(  s(k+1)  ) ) )
							{
								e(k) = zero;
								break;
							}
						}
						if (k == p-2)
						{
							kase = 4;
						}
						else
						{
							int ks;
							for (ks = p-1; ks >= k; --ks)
							{
								if (ks == k)
								{
									break;
								}
								
								value_type t =  (ks != p ? fabs( e(ks) ) : zero )  + (ks != k+1 ? fabs( e(ks-1) ) : zero);
								
								if (fabs( s(ks)  ) <= eps*t)
								{
									s(ks) = zero;
									break;
								}
								
							}
							
							if (ks == k) 
							{
								kase = 3;
							}
							else if (ks == p-1) 
							{
								kase = 1;
							}
							else
							{
								kase = 2;
								k = ks;
							}
						}
						++k;
						
						// Perform the task indicated by kase.
						switch (kase) 
						{
								
								// Deflate negligible s(p).
							case 1: 
							{
								value_type f = e(p-2);
								
								e(p-2) = zero;
								
								for (int j = p-2; j >= k; --j)
								{
									value_type t  = hypot(  s(j), f);
									value_type cs = s(j) / t;
									value_type sn = f / t;
									
									s(j) = t;
									
									if (j != k) 
									{
										f      = -sn*e(j-1);
										e(j-1) =  cs*e(j-1);
									}
									if (wantv) 
									{
										for (int i = 0; i < n; ++i)
										{
											t        =  cs*V(i,j) + sn*V(i,p-1);
											V(i,p-1) = -sn*V(i,j) + cs*V(i,p-1);
											V(i,j)   = t;
										}
									}
								}
							}
								break;
								
								// Split at negligible s(k).
							case 2: 
							{
								value_type f = e(k-1);
								e(k-1) = zero;
								for (int j = k; j < p; ++j)
								{
									value_type t  =  hypot( s(j), f);
									value_type cs =  s(j) / t;
									value_type sn =  f / t;
									s(j) = t;
									f    = -sn*e(j);
									e(j) =  cs*e(j);
									if (wantu) 
									{
										for (int i = 0; i < m; ++i) 
										{
											t        =  cs*U(i,j) + sn*U(i,k-1);
											U(i,k-1) = -sn*U(i,j) + cs*U(i,k-1);
											U(i,j)   = t;
										}
									}
								}
							}
								break;
								
								// Perform one qr step.
							case 3: 
							{
								// Calculate the shift.							
								value_type scale = max(max(max(max(   fabs(s(p-1)),fabs(s(p-2))),fabs(e(p-2))), fabs(s(k))), fabs(e(k)));
								
								value_type sp    = s(p-1) / scale;
								value_type spm1  = s(p-2) / scale;
								value_type epm1  = e(p-2) / scale;
								value_type sk    = s(k)   / scale;
								value_type ek    = e(k)   / scale;
								value_type b     = ((spm1 + sp)*(spm1 - sp) + epm1*epm1) / two;
								value_type c     = (sp*epm1)*(sp*epm1);
								value_type shift = zero;
								if ((b != zero) || (c != zero)) 
								{
									shift = sqrt(b*b + c);
									if ( b < zero ) 
									{
										shift = -shift;
									}
									shift = c/(b + shift);
								}
								value_type f = (sk + sp)*(sk - sp) + shift;
								value_type g = sk*ek;
								
								// Chase zeros.
								for (int j = k; j < p-1; ++j) 
								{
									value_type t  = hypot(f,g);
									value_type cs = f/t;
									value_type sn = g/t;
									if (j != k) 
									{
										e(j-1) = t;
									}
									f      = cs*s(j) + sn*e(j);
									e(j)   = cs*e(j) - sn*s(j);
									g      = sn*s(j+1);
									s(j+1) = cs*s(j+1);
									if (wantv) 
									{
										for (int i = 0; i < n; ++i) 
										{
											t        =  cs*V(i,j) + sn*V(i,j+1);
											V(i,j+1) = -sn*V(i,j) + cs*V(i,j+1);
											V(i,j)   = t;
										}
									}
									t    = hypot(f,g);
									cs   = f/t;
									sn   = g/t;
									s(j) = t;
									f      =  cs*e(j) + sn*s(j+1);
									s(j+1) = -sn*e(j) + cs*s(j+1);
									g      = sn*e(j+1);
									e(j+1) = cs*e(j+1);
									if (wantu && (j < m-1)) 
									{
										for (int i = 0; i < m; ++i)
										{
											t        =  cs*U(i,j) + sn*U(i,j+1);
											U(i,j+1) = -sn*U(i,j) + cs*U(i,j+1);
											U(i,j)   = t;
										}
									}
								}
								e(p-2) = f;
								iteration = iteration + 1;
							}
								break;
								
								// Convergence.
							case 4: 
							{
								// Make the singular values positive.
								if (s(k) <= zero) 
								{
									s(k) = (s(k) < zero ? -s(k) : zero);
									if (wantv) 
									{
										for (int i = 0; i <= pp; ++i) 
										{
											V(i,k) = -V(i,k);
										}
									}
								}
								// Order the singular values.
								while (k < pp) 
								{
									if (s(k) >= s(k+1)) 
									{
										break;
									}
									value_type t = s(k);
									s(k)   = s(k+1);
									s(k+1) = t;
									if (wantv && (k < n-1)) 
									{
										for (int i = 0; i < n; ++i) 
										{
											t        = V(i,k+1);
											V(i,k+1) = V(i,k);
											V(i,k)   = t;
										}
									}
									if (wantu && (k < m-1)) 
									{
										for (int i = 0; i < m; ++i)
										{
											t        = U(i,k+1);
											U(i,k+1) = U(i,k);
											U(i,k)   = t;
										}
									}
									++k;
								}
								iteration = 0;
								--p;
							}
								break;
						}
					}
				}
				
			} // namespace detail
			
			
		} // namespace big
	} // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_SVD_IMPL1_H
#endif
