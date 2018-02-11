#ifndef OPENTISSUE_CORE_MATH_MATH_ROTATION_H
#define OPENTISSUE_CORE_MATH_MATH_ROTATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/math/math_quaternion.h>
#include <OpenTissue/core/math/math_value_traits.h>

#include <cmath>
#include <iosfwd>


namespace OpenTissue
{

  namespace math
  {

  template<
      typename value_type_
      //, typename value_traits_ = ValueTraits<value_type_> 
    >
    class Rotation
    {
    protected:

      typedef typename OpenTissue::math::ValueTraits<value_type_>  value_traits_ ;  // TODO value_traits_ should be parameterized as a class template parameter.

    public:

      typedef value_traits_           value_traits;  ///< Convinience typedef to make value traits accesible for all template functions using Vector3 types.
      typedef value_type_             value_type;
      typedef Matrix3x3<value_type>   matrix3x3_type;
      typedef Vector3<value_type>     vector3_type;
      typedef Quaternion<value_type>  quaternion_type;

    protected:

      value_type     m_angle;     ///< The Rotation angle in radians. Default value is zero.
      vector3_type   m_axis;      ///< The Rotation axe. This should be an unit vector.

    public:

      value_type       & angle()       { return m_angle; }
      value_type const & angle() const { return m_angle; }

      vector3_type       & axis()       { return m_axis; }
      vector3_type const & axis() const { return m_axis; }

    public:

      Rotation()
        : m_angle(value_traits::zero()) 
        , m_axis(value_traits::zero(),value_traits::zero(),value_traits::one())
      {}

      explicit Rotation(quaternion_type const & q) { *this = q; }

      explicit Rotation(matrix3x3_type const & R) { *this = R; }

      ~Rotation(){}

      Rotation & operator=(Rotation const & R)
      {
        m_angle = R.m_angle;
        m_axis  = R.m_axis;
        return (*this);
      }

      Rotation & operator=(quaternion_type const & q)
      {
        using std::acos;
        using std::sin;

        value_type teta     = boost::numeric_cast<value_type>( acos( q.s() ) );
        value_type sin_teta = boost::numeric_cast<value_type>( sin(teta)     );
        m_angle = value_traits::two()*teta;
        if(sin_teta)
          m_axis = unit( q.v() / sin_teta );
        else
          m_axis = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one());
        return *this;
      }

      Rotation & operator=(matrix3x3_type const & M)
      {
        Quaternion<value_type> tmp(M);
        (*this) = tmp;
        return *this;
      }

    };

    template<typename T>
    inline std::ostream & operator<< (std::ostream & o, Rotation<T> const & r)
    {
      o << "[" << r.angle() << "," << r.axis()(0) << "," << r.axis()(1) << "," << r.axis()(2) << "]";
      return o;
    }

    template<typename T>
    inline std::istream & operator>>(std::istream & i, Rotation<T> & r)
    {
      char dummy;
      i >> dummy;
      i >> r.angle();
      i >> dummy;
      i >> r.axis()(0);
      i >> dummy;
      i >> r.axis()(1);
      i >> dummy;
      i >> r.axis()(2);
      i >> dummy;
      return i;
    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_ROTATION_H
#endif
