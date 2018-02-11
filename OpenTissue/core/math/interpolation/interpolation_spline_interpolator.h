#ifndef OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_SPLINE_INTERPOLATOR_H
#define OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_SPLINE_INTERPOLATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interpolation/interpolation_base_interpolator.h>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{

  namespace interpolation
  {

    /**
    * Spline Interpolation.
    * This is an implementation of the algorithm, which is
    * described in Numerical Recipes in C, pp. 113-116.
    *
    */
    template<typename real_type_>
    class SplineInterpolator : public BaseInterpolator<SplineInterpolator<real_type_>, real_type_ >
    {
    public:

      typedef typename real_type_    real_type;

    private:

      real_type * m_y2;    ///< Temporary Working Storage. Used by the method setTableau() for holding the derivatives of the spline.
      real_type * m_u;     ///< Temporary Working Storage.Used by the method setTableau(). Used for intermediate calculations.
      int         m_n;     ///< The length of the tableau's
      int         m_N;     ///< The maximum avaible length of the tableau's
      real_type * m_xa;    ///< Local Reference, used by method getValue().
      real_type * m_y2a;   ///< Local Reference, used by method getValue().
      real_type * m_ya;    ///< Local Reference, used by method getValue().

    public:


      SplineInterpolator()
        :m_y2(0)
        ,m_u(0)
        ,m_n(0)
        ,m_N(0)
        ,m_xa(0)
        ,m_y2a(0)
        ,m_ya(0)
      {}

      ~SplineInterpolator()
      {
        if(m_y2)
          delete [] m_y2;
        if(m_u)
          delete [] m_u;
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
        set_tableau(x,y,cnt, math::detail::highest<real_type>(), math::detail::highest<real_type>() );
      }

      /**
      *
      */
      void set_tableau(real_type * x,real_type * y,int cnt,real_type yDerivStart,real_type yDerivEnd)
      {
        //--- Allocate temporary working storage etc.
        if(cnt>m_N)
        {
          m_N = cnt;
          if(m_y2)
            delete [] m_y2;
          m_y2 = new real_type[m_N];
          if(m_u)
            delete [] m_u;
          m_u = new real_type[m_N];
        }
        this ->m_n = cnt;
        //--- Setup references used by method getValue
        m_xa  = x;
        m_ya  = y;
        m_y2a = m_y2;


        real_type p,qn,sig,un;

        if(yDerivStart >= math::detail::highest<real_type>() )
          m_y2[0]=m_u[0]=static_cast<real_type>(0.0);
        else
        {
          m_y2[0] = static_cast<real_type>(-0.5);
          m_u[0]=(3.0f/(x[1]-x[0]))*((y[1]-y[0])/(x[1]-x[0])-yDerivStart);
        }
        for(int i=1;i<=m_n-2;++i)
        {
          sig     = (x[i]-x[i-1])/(x[i+1]-x[i-1]);
          p       = sig*m_y2[i-1] + 2.0;
          m_y2[i] = (sig-1.0)/p;
          m_u[i]  =(y[i+1]-y[i])/(x[i+1]-x[i]) - (y[i]-y[i-1])/(x[i]-x[i-1]);
          m_u[i]  =(6.0f*m_u[i]/(x[i+1]-x[i-1])-sig*m_u[i-1])/p;
        }
        if(yDerivEnd >= math::detail::highest<real_type>() )
          qn=un=static_cast<real_type>(0.0);
        else
        {
          qn= static_cast<real_type>(0.5);
          un=(3.0f/(x[m_n-1]-x[m_n-2]))*(yDerivEnd-(y[m_n-1]-y[m_n-2])/(x[m_n-1]-x[m_n-2]));
        }
        m_y2[m_n-1]=(un-qn*m_u[m_n-2])/(qn*m_y2[m_n-2]+1.0);

        for(int k=m_n-2;k>=0;--k)
          m_y2[k] =m_y2[k]*m_y2[k+1]+m_u[k];
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
        int klo,khi,k;
        real_type h,b,a;

        klo=0;
        khi=m_n-1;

        while (khi-klo > 1)
        {
          k=(khi+klo) >> 1;
          if (m_xa[k] > x) khi=k;
          else klo=k;
        }
        h=m_xa[khi]-m_xa[klo];
        // TODO: Comparing floats with == or != is not safe
        if(std::fabs(h)> static_cast<real_type>(0.0) )
        {
          std::cerr << "SplineInterpolator::get_value(...): Bad x-array input in method set_tableau()" << std::endl;
        }
        a=(m_xa[khi]-x)/h;
        b=(x-m_xa[klo])/h;

        real_type y = a*m_ya[klo]+b*m_ya[khi]+((a*a*a-a)*m_y2a[klo]+(b*b*b-b)*m_y2a[khi])*(h*h)/6.0;

        return y;
      }

      /**
      * Error Estimate Method.
      *
      * @return   The value of the error estimate.
      */
      real_type get_error_estimate()const {  return static_cast<real_type>(0.0);}

    };

  } // namespace interpolation

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_SPLINE_INTERPOLATOR_H
#endif
