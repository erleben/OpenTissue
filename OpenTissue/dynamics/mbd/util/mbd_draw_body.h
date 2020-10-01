#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_BODY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_BODY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/graphics/core/gl/gl_util.h>

#include <OpenTissue/collision/sdf/sdf.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/geometry/geometry_obb.h>


namespace OpenTissue
{
  namespace mbd
  {

    template<typename body_type>
    void draw_body(body_type const & body, bool wireframe)
    {
      typedef typename body_type::math_policy           math_policy;

      typedef typename body_type::value_traits          value_traits;
      typedef typename body_type::node_traits           node_traits;

      typedef typename math_policy::real_type             real_type;
      typedef typename math_policy::vector3_type          vector3_type;
      typedef typename math_policy::matrix3x3_type        matrix3x3_type;
      typedef typename math_policy::quaternion_type       quaternion_type;

      typedef OpenTissue::geometry::Sphere<math_policy>       sphere_type;
      typedef OpenTissue::geometry::Plane<math_policy>        plane_type;
      typedef OpenTissue::geometry::OBB<math_policy>          box_type;
      typedef OpenTissue::polymesh::PolyMesh<math_policy>     mesh_type;
      typedef OpenTissue::grid::Grid<float,math_policy>       grid_type;
      typedef OpenTissue::sdf::Geometry<mesh_type,grid_type>  sdf_geometry_type;

      static real_type const one_third = value_traits::one() / 3;
      static real_type const two_third = value_traits::two()*one_third;

      if(body.is_fixed())
        gl::ColorPicker(one_third,one_third,one_third);
      else if (body.is_sleepy())
        gl::ColorPicker(value_traits::zero(),value_traits::zero(),two_third);
      else
        gl::ColorPicker(two_third,two_third,two_third);

      //if(body.m_sa_stack_height==0)
      //  gl::ColorPicker(one_third,one_third,one_third);
      //else if(body.m_sa_stack_height==1)
      //  gl::ColorPicker(one_third,value_traits::zero(),value_traits::zero());  // red
      //else if(body.m_sa_stack_height==2)
      //  gl::ColorPicker(value_traits::zero(),one_third,value_traits::zero()); // green
      //else if(body.m_sa_stack_height==3)
      //  gl::ColorPicker(value_traits::zero(),value_traits::zero(),one_third);  // blue
      //else if(body.m_sa_stack_height==4)
      //  gl::ColorPicker(one_third,one_third,value_traits::zero());  // yellow
      //else if(body.m_sa_stack_height==5)
      //  gl::ColorPicker(one_third, value_traits::zero(), one_third, value_traits::zero()); // puple
      //else
      //  gl::ColorPicker(value_traits::zero(),value_traits::zero(),value_traits::zero());

      glPushMatrix();
      vector3_type r;
      quaternion_type Q;
      body.get_position(r);
      body.get_orientation(Q);
      gl::Transform(r,Q);

      // 2008-06-13 kenny: this is down-right ugly! Stuf like this should
      // not be part of a multibody simulator! And if functionality like
      // this should be supported it would proberly be better to do it with
      // virtual interfaces or using a dispatcher.
      if(body.get_geometry()->class_id() == sdf_geometry_type::id() )
      {
        sdf_geometry_type * sdf = static_cast<sdf_geometry_type*>(body.get_geometry());
        if(wireframe)
          gl::DrawMesh(sdf->m_mesh,GL_LINE_LOOP);
        else
          gl::DrawMesh(sdf->m_mesh,GL_POLYGON);
        gl::ColorPicker(value_traits::one(),value_traits::zero(),value_traits::zero());
        gl::DrawGridAABB(sdf->m_phi);
      }
      else if(body.get_geometry()->class_id() == box_type::id() )
      {
        gl::DrawOBB( *(static_cast<box_type*>(body.get_geometry() )), wireframe);
      }
      else if(body.get_geometry()->class_id() == sphere_type::id() )
      {
        gl::DrawSphere( *(static_cast<sphere_type*>(body.get_geometry() )), wireframe);
      }
      else if(body.get_geometry()->class_id() == plane_type::id() )
      {
        gl::DrawPlane(  *(static_cast<plane_type*>(body.get_geometry() )), wireframe );
      }
      else 
      {
        assert(!"draw_body(): not handled");
      }
      glPopMatrix();
      if(wireframe)
      {
        vector3_type u;
        body.get_velocity(u);
        gl::ColorPicker(value_traits::one(),value_traits::zero(),value_traits::one());
        gl::DrawVector(r,u);
        gl::ColorPicker(value_traits::one(),value_traits::one(),value_traits::zero());
        body.get_spin(u);
        gl::DrawVector(r,u);
      }
    }

    template<bool wireframe>
    class DrawBodyFunctor
    {
    public:
      template<typename body_type>
      void operator()(body_type const & body) const
      {
        draw_body(body,false);
      }
    };

    template<>
    class DrawBodyFunctor<true>
    {
    public:
      template<typename body_type>
      void operator()(body_type const & body) const
      {
        draw_body(body,true);
      }
    };


  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_BODY_H
#endif 
