#ifndef OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_POLYNOMIAL_INTERPOLATOR_H
#define OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_POLYNOMIAL_INTERPOLATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interpolation/interpolation_base_interpolator.h>


namespace OpenTissue
{

  namespace interpolation
  {

    /**
    * Polynomial Interpolator.
    * This is an implementation of the algorithm, which is
    * described in Numerical Recipes in C, pp. 108-110.
    *
    * The original algorithm as it stands in the book have a small
    * error regarding how it tracks its way through the tableau (try
    * to watch what happens if you extrapolate on the right side, that
    * is only want to pick D's). We have corrected this error in our
    * implementation.
    */
    template<typename real_type_>
    class PolynomialInterpolator : public BaseInterpolator< PolynomialInterpolator<real_type_>, real_type_ >
    {
    public:

      typedef real_type_    real_type;

    private:

      real_type   m_error; ///< Placeholder for the current error esitimate.
      int         m_n;     ///< The current order of the interpolation.
      real_type * m_xa;    ///< A reference to the table of x-values
      real_type * m_ya;    ///< A reference to the table of y-values, such that yi = f[xi]
      real_type * m_C;     ///< Internal working storage, needed by the method getValue()
      real_type * m_D;     ///< Internal working storage, needed by the method getValue()

    public:

      PolynomialInterpolator()
        : m_error(0.0)
        , m_n(-1)
        , m_xa(0)
        , m_ya(0)
        , m_C(0)
        , m_D(0)
      {}

      ~PolynomialInterpolator()
      {
        if(m_C)
          delete [] m_C;
        if(m_D)
          delete [] m_D;
      }

      /**
      * Set Tableau Method.
      *
      * @param x   The array of parameter values.
      * @param y   The array of corresponding function values. Must
      *            have same length as the x-array.
      */
      void set_tableau(real_type * x,real_type * y,int cnt)
      {
        //--- Assign local data members
        this->m_xa = x;
        this->m_ya = y;

        //--- (Lazy) Allocate internal storage needed during the computation
        if(m_C==0)
        {
          m_C = new real_type[cnt];
          m_D = new real_type[cnt];
        }
        else if(cnt<m_n)
        {
          delete [] m_C;
          m_C = new real_type[cnt];
          delete [] m_D;
          m_D = new real_type[cnt];
        }
        this->m_n = cnt;
      }

      /**
      * Get Interpolation Value.
      *
      * @param x    The point at which the interpolated or extrapolated
      *             value should be found at.
      *
      * @return     The interpolated (or extrapolated) value at
      *             the point x.
      */
      real_type get_value(real_type const & x)
      {
        //--- First we find the the closest table
        //--- entry index to x and initialize tables
        //--- C and D at the same time.
        real_type diff = static_cast<real_type>( std::fabs(x-m_xa[0]) );
        int ns = 0;
        m_D[0] = m_C[0] = m_ya[0];
        for(int i=1;i<m_n;++i)
        {
          real_type test = static_cast<real_type>( std::fabs(x-m_xa[i]) );
          if(test<diff)
          {
            ns =  i;
            diff = test;
          }
          m_C[i] = m_D[i] = m_ya[i];
        }

        //--- This is the initial approximation to y=f(x).
        real_type y = m_ya[ns];     //---- This was original ya[ns--]

        for(int m=1;m<m_n;++m)
        {
          for(int i=0;i<(m_n-m);++i)
          {
            real_type ho = m_xa[i] - x;
            //--- You might wonder why the line below doesn't say
            //---   hp = xa[i+m+1]-x
            //--- This is because the addition with the constant 1
            //--- have been moved to the outer loop
            real_type hp = m_xa[i+m] - x;
            real_type w = m_C[i+1]-m_D[i];
            real_type denominator= ho-hp;
            // TODO: Comparing floats with == or != is not safe
            if(denominator==0)
            {
              //--- This error can occur only if two input x's are (to within roundoff) identical.
              std::cerr << "PolynomialInterpolator::get_value(): Unexpected internal error, unable to compute!" << std::endl;
              return static_cast<real_type>(0.0);
            }
            denominator = w / denominator;
            //---
            //---              ( x(i+m+1) - x )  ( C(m,i+1) - D(m,i) )
            //---  D(m+1,i) = -----------------------------------------
            //---                         x(i) - x(i+m+1)
            //---
            m_D[i] = hp*denominator;
            //---
            //---              ( x(i) - x )  ( C(m,i+1) - D(m,i) )
            //---  C(m+1,i) = -------------------------------------
            //---                       x(i) - x(i+m+1)
            //---
            m_C[i] = ho*denominator;
          }
          //---
          //--- The value of (n-m) is the current number of rows
          //--- in the tableaus of C and D
          //---
          if((2*ns)<(m_n-m))
          {
            m_error = m_C[ns];        //--- This was original C[ns+1]
          }
          else
          {
            ns = ns - 1;
            m_error = m_D[ns];       //--- This was original D[ns--]
          }
          y += m_error;
        }

        return y;
      }

      /**
      * Error Estimate Method.
      *
      * @return   The value of the error estimate.
      */
      real_type get_error_estimate()const {  return m_error;}

    };

  } // namespace interpolation

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_POLYNOMIAL_INTERPOLATOR_H
#endif
