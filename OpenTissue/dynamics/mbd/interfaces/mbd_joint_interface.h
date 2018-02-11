#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_JOINT_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_JOINT_INTERFACE_H
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
    * joint_type Class.
    *
    * Not all joints, limits and motors are compatible, so a type
    * check must be performed when attaching a limit or motor to a
    * joint to ensure that the modelling is consistent.
    */
    template< typename mbd_types >
    class JointInterface 
      : public mbd_types::identifier_type
      , public mbd_types::constraint_type
    {
    public:

      typedef typename mbd_types::socket_type   socket_type;

    protected:

      socket_type * m_socketA;
      socket_type * m_socketB;

    public:

      JointInterface()
        : m_socketA(0)
        , m_socketB(0)
      {}

      virtual ~JointInterface() {   clear(); }

    public:

      socket_type       * get_socket_A()       { return m_socketA; }
      socket_type       * get_socket_B()       { return m_socketB; }
      socket_type const * get_socket_A() const { return m_socketA; }
      socket_type const * get_socket_B() const { return m_socketB; }

      void connect( socket_type const & socketA, socket_type const & socketB )
      {
        assert(!this->m_bodyA || !"JointInterface::connect(): body A was non-null");
        assert(!this->m_bodyB || !"JointInterface::connect(): body B was non-null");
        assert(socketA.get_body() || !"JointInterface::connect(): body from socket A was null");
        assert(socketB.get_body() || !"JointInterface::connect(): body from socket B was null");
        assert(socketA.get_body()!=socketB.get_body()  || !"JointInterface::connect(): bodies from sockets were the same");

        m_socketA = const_cast<socket_type*>(&socketA);
        m_socketB = const_cast<socket_type*>(&socketB);

        this->m_bodyA = m_socketA->get_body();
        this->m_bodyB = m_socketB->get_body();

        this->m_bodyA->m_joints.push_back(this);
        this->m_bodyB->m_joints.push_back(this);

        calibration();
      }

      void disconnect()
      {
        if(this->m_bodyA)
          this->m_bodyA->m_joints.remove(this);
        if(this->m_bodyB)
          this->m_bodyB->m_joints.remove(this);
        m_socketA = 0;
        m_socketB = 0;
        this->m_bodyA = 0;
        this->m_bodyB = 0;
      }

      /**
      * joint_type Calibration Routine.
      * This method is invoked implicitly whenever the joint connectivity
      * is changed (alteration of anchor points or joint axes).
      *
      * It can also be invoked directly by and end user if the
      * incident body position and/or rotations is changed.
      */
      virtual void calibration()=0;

      void clear() 
      { 
        disconnect(); 
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_JOINT_INTERFACE_H
#endif
