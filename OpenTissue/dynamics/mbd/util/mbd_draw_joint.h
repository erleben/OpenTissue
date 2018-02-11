#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_JOINT_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_JOINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/gl/gl_util.h>

namespace OpenTissue
{
  namespace mbd
  {

    template<typename joint_type>
    void draw_joint(joint_type const & joint)
    {
      typedef typename joint_type::socket_type                socket_type;
      typedef typename joint_type::body_type                  body_type;
      typedef typename body_type::real_type                   real_type;
      typedef typename body_type::vector3_type                vector3_type;
      typedef typename body_type::matrix3x3_type              matrix3x3_type;
      typedef typename body_type::quaternion_type             quaternion_type;

      vector3_type r;
      quaternion_type Q;

      glPushMatrix();
      joint.get_socket_A()->get_body()->get_position(r);
      joint.get_socket_A()->get_body()->get_orientation(Q);
      gl::Transform(r,Q);
      gl::DrawFrame(joint.get_socket_A()->get_joint_frame());
      glPopMatrix();

      glPushMatrix();
      joint.get_socket_B()->get_body()->get_position(r);
      joint.get_socket_B()->get_body()->get_orientation(Q);
      gl::Transform(r,Q);
      gl::DrawFrame(joint.get_socket_B()->get_joint_frame());
      glPopMatrix();
    }

    class DrawJointFunctor
    {
    public:
      template<typename joint_type>
      void operator()(joint_type const & joint) const
      {
        draw_joint(joint);
      }
    };

  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_DRAW_JOINT_H
#endif 
