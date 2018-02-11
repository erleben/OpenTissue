#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_JOINT_SOCKET_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_JOINT_SOCKET_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * A joint_type Socket.
    * A socket is a joint coordinate frame together with the body
    * on which the joint coordinate frame is placed.
    */
    template<typename mbd_types>
    class JointSocket
    {
    public:

      typedef typename mbd_types::body_type                      body_type;
      typedef typename mbd_types::socket_type                    socket_type;
      typedef typename mbd_types::math_policy::vector3_type      vector3_type;
      typedef typename mbd_types::math_policy::matrix3x3_type    matrix3x3_type;
      typedef typename mbd_types::math_policy::quaternion_type   quaternion_type;
      typedef typename mbd_types::math_policy::coordsys_type     coordsys_type;

    protected:

      body_type     * m_body;           ///< A pointer to the body on which the joint coordinate frame is placed.
      coordsys_type   m_joint_frame;    ///< Placement of joint frame in the body frame.

    public:

      JointSocket()
        : m_body(0)
      {}

    public:

      void init(body_type const & body, coordsys_type const & joint_frame)
      {
        m_body = const_cast<body_type*>(&body);
        m_joint_frame = joint_frame;
      }

      body_type       * get_body()       { return m_body; }
      body_type const * get_body() const { return m_body; }

      vector3_type get_anchor_local()     const { return m_joint_frame.T();                             }
      vector3_type get_axis1_local()      const { return m_joint_frame.Q().rotate(vector3_type(1,0,0)); }
      vector3_type get_axis2_local()      const { return m_joint_frame.Q().rotate(vector3_type(0,1,0)); }
      vector3_type get_joint_axis_local() const { return m_joint_frame.Q().rotate(vector3_type(0,0,1)); }

      vector3_type get_anchor_world() const
      {
        assert(m_body || !"JointSocket::get_anchor_world(): body was null");
        matrix3x3_type R;
        vector3_type r;
        m_body->get_orientation(R);
        m_body->get_position(r);
        return (R*m_joint_frame.T() + r);
      }

      vector3_type get_axis1_world() const
      {
        assert(m_body || !"JointSocket::get_axis1_world(): body was null");
        matrix3x3_type R;
        m_body->get_orientation(R);
        return R*get_axis1_local();
      }

      vector3_type get_axis2_world() const
      {
        assert(m_body || !"JointSocket::get_axis2_world(): body was null");
        matrix3x3_type R;
        m_body->get_orientation(R);
        return R*get_axis2_local();
      }

      vector3_type get_joint_axis_world() const
      {
        assert(m_body || !"JointSocket::get_axis2_world(): body was null");
        matrix3x3_type R;
        m_body->get_orientation(R);
        return R*get_joint_axis_local();
      }

      coordsys_type get_joint_frame() const { return m_joint_frame; }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_JOINT_SOCKET_H
#endif
