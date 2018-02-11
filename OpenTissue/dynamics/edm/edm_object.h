#ifndef OPENTISSUE_DYNAMICS_EDM_EDM_OBJECT_H
#define OPENTISSUE_DYNAMICS_EDM_EDM_OBJECT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>
#include <OpenTissue/core/geometry/geometry_base_shape.h>
#include <OpenTissue/core/function/function_signed_distance_function.h>

#include <cassert>


namespace OpenTissue
{

  namespace edm
  {

    /**
     * An Implicit Impenetrable Object (obstacle)
     */
    template<typename edm_types>
    class Object
      : public edm_types::object_traits
    {
    public:

      typedef typename edm_types::value_traits              value_traits;
      typedef typename edm_types::math_types                math_types;
      typedef typename edm_types::real_type                 real_type;
      typedef typename edm_types::vector3_type              vector3_type;
      typedef geometry::BaseShape<math_types>               base_shape;
      typedef function::SignedDistanceFunction<math_types>  sdf_type;

    protected:
      
      bool              m_visible;
      real_type         m_scale;
      base_shape     *  m_shape;
      sdf_type const *  m_sdf;
      
    public:

      Object()
        : m_visible(true)
        , m_scale(value_traits::one())
        , m_shape(0)
        , m_sdf(0)
      {}

      virtual ~Object()
      {
        delete m_shape;
      }

    public:

      template<typename shape_type>
      shape_type& create_shape()
      {
        shape_type* shape = new shape_type;
        // 2009-03-11 kenny: why dynamic cast? could we not do without?
        m_shape = dynamic_cast<base_shape*>(shape);
        assert(m_shape || !"shape_type does not support the ShapeBase interface!");
        m_sdf = dynamic_cast<sdf_type*>(m_shape);
        assert(m_sdf || !"shape_type does not support the SignedDistanceFunction interface!");
        return *static_cast<shape_type*>(m_shape);
      }

      base_shape const * get_shape() const
      {
        return m_shape;
      }

      Object & set_scale(real_type const & scale)
      {
        m_scale = scale;
        return *this;
      }

      real_type const & get_scale() const
      {
        return m_scale;
      }

      Object & set_visibility(bool visible)
      {
        m_visible = visible;
        return *this;
      }

      bool get_visibility() const
      {
        return m_visible;
      }

    public:

      real_type eval(vector3_type const & x) const  ///< the object's inside/outside function
      {
        // 2009-03-11 kenny: assert without proper message
        assert(m_sdf);
        return real_type(m_sdf->evaluate(x));
      }

      vector3_type normal(vector3_type const & x) const  ///< the object's gradient vector at a point
      {
        // 2009-03-11 kenny: assert without proper message
        assert(m_sdf);
        return vector3_type(m_sdf->normal(x));
      }

      real_type dist(vector3_type const & x) const  ///< the distace from p to the surface
      {
        using std::fabs;
        // 2009-03-11 kenny: assert without proper message
        assert(m_sdf);
        return real_type(fabs(m_sdf->signed_distance(x)));
      }

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_EDM_OBJECT_H
#endif
