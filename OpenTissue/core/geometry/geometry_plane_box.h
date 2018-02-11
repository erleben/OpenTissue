#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PLANE_BOX_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PLANE_BOX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/core/geometry/geometry_plane.h>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * A Plane Box class.
    * This class creates a plane-box. A plane-box is an interactive
    * tool for manipulating an axis-parallel plane.
    *
    * Such an interactive plane can be quite usefull in visualization of
    * various geometric structures. For instace a tetrahedra mesh can benefit
    * from it as shown in the example below.
    *
    * Example usage
    *
    *  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
    *  typedef OpenTissue::PlaneBox<math_types>  plane_box_type;
    *
    *  plane_box_type m_plane_box;
    *
    *  void do_display()
    *  {
    *    OpenTissue::gl::ColorPicker(0.8,0.1,0.8);
    *    OpenTissue::gl::DrawPlaneBox( m_plane_box );
    *    OpenTissue::gl::ColorPicker(0.1,0.8,0.8);
    *    OpenTissue::gl::DrawT4MeshCutThrough(my_t4mesh,m_plane_box.plane(),0.95,false);
    *  }
    *
    *  void do_action(unsigned char choice)
    *  {
    *    switch ( choice )
    *    {
    *      case 'i': // initialize
    *      {
    *        vector3_type min_coord;
    *        vector3_type max_coord;
    *        ....
    *        m_plane_box.init( min_coord, max_coord);
    *      }
    *      break;
    *      case '\t': // toggle plane
    *      {
    *        static int axis = 0;
    *        axis = (axis+1)%3;
    *        if(axis==0)
    *          m_plane_box.set_x_axis();
    *        if(axis==1)
    *          m_plane_box.set_y_axis();
    *        if(axis==2)
    *          m_plane_box.set_z_axis();
    *      }
    *      break;
    *      case '+':
    *      {
    *        m_plane_box.increment();
    *      }
    *      break;
    *      case '-':
    *        {
    *          m_plane_box.decrement();
    *        }
    *        break;
    *      default:
    *        std::cout << "You pressed " << choice << std::endl;    break;
    *      };
    *    }
    */
    template<typename math_types>
    class PlaneBox
    {
    public:

      typedef typename math_types::value_traits      value_traits;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::real_type         real_type;
      typedef Plane<math_types>                      plane_type;

    protected:

      typedef AABB<math_types>                       aabb_type;

      aabb_type   m_box;
      plane_type  m_plane;
      int         m_max_tick;       ///< The maximum ticks that a side along the box should be divided into (i.e. the number of plane positions).
      int         m_cur_tick;       ///< The current tick value along the m_axis direction that the plane should be placed at.
      int         m_axis;           ///< Indicator of which axis the plane is orthogonal to.


      vector3_type m_p[4][3];      ///< Base points of rectangle representing plane
      vector3_type m_n[3];         ///< Possible normal directions, current direction is chosen by value of m_axis.

    public:

      plane_type   const & plane() const { return m_plane;       }
      aabb_type    const & box()   const { return m_box;         }
      vector3_type const & p0()    const { return m_p[0][m_axis];}
      vector3_type const & p1()    const { return m_p[1][m_axis];}
      vector3_type const & p2()    const { return m_p[2][m_axis];}
      vector3_type const & p3()    const { return m_p[3][m_axis];}
      vector3_type const &  n()    const { return m_n[m_axis];   }

    public:

      PlaneBox()
        : m_max_tick(100)
        , m_cur_tick(50)
        , m_axis(0)
      {
        m_n[0] = vector3_type(1,0,0);
        m_n[1] = vector3_type(0,1,0);
        m_n[2] = vector3_type(0,0,1);
      }

      void init( vector3_type const & min_coord, vector3_type const & max_coord )
      {
        m_box.set(min_coord, max_coord);
        m_cur_tick = m_max_tick/2;
        update();
      }

      void increment()
      {
        ++m_cur_tick;
        if(m_cur_tick>m_max_tick)
          m_cur_tick = m_max_tick;
        update();
      }

      void decrement()
      {
        --m_cur_tick;
        if(m_cur_tick<0)
          m_cur_tick = 0;
        update();
      }

      void set_x_axis(){ m_axis = 0; update(); }
      void set_y_axis(){ m_axis = 1; update(); }
      void set_z_axis(){ m_axis = 2; update(); }

    protected:

      void update()
      {
        real_type fraction = (value_traits::one()*m_cur_tick / m_max_tick);

        // x-base
        m_p[0][0] = m_box.min();
        m_p[1][0] = m_box.min() + vector3_type( value_traits::zero(), m_box.h(),            value_traits::zero() );
        m_p[2][0] = m_box.min() + vector3_type( value_traits::zero(), m_box.h(),            m_box.d()            );
        m_p[3][0] = m_box.min() + vector3_type( value_traits::zero(), value_traits::zero(), m_box.d()            );

        // y-base
        m_p[0][1] = m_box.min();
        m_p[1][1] = m_box.min() + vector3_type( value_traits::zero(), value_traits::zero(), m_box.d()            );
        m_p[2][1] = m_box.min() + vector3_type( m_box.w(),            value_traits::zero(), m_box.d()            );
        m_p[3][1] = m_box.min() + vector3_type( m_box.w(),            value_traits::zero(), value_traits::zero() );

        // z-base
        m_p[0][2] = m_box.min();
        m_p[1][2] = m_box.min() + vector3_type( m_box.w(),            value_traits::zero(), value_traits::zero() );
        m_p[2][2] = m_box.min() + vector3_type( m_box.w(),            m_box.h(),            value_traits::zero() );
        m_p[3][2] = m_box.min() + vector3_type( value_traits::zero(), m_box.h(),            value_traits::zero() );

        m_p[0][0](0) += m_box.w()*fraction;
        m_p[1][0](0) += m_box.w()*fraction;
        m_p[2][0](0) += m_box.w()*fraction;
        m_p[3][0](0) += m_box.w()*fraction;

        m_p[0][1](1) += m_box.h()*fraction;
        m_p[1][1](1) += m_box.h()*fraction;
        m_p[2][1](1) += m_box.h()*fraction;
        m_p[3][1](1) += m_box.h()*fraction;

        m_p[0][2](2) += m_box.d()*fraction;
        m_p[1][2](2) += m_box.d()*fraction;
        m_p[2][2](2) += m_box.d()*fraction;
        m_p[3][2](2) += m_box.d()*fraction;

        m_plane.set( m_n[m_axis], m_p[0][m_axis]);

      }

    };

  } // namespace geometry
} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PLANE_BOX_H
#endif



