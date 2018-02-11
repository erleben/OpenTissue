#ifndef DATA_H
#define DATA_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include "types.h"

#include <vector>


class Data
{
public:

  gravity_type                m_gravity;
  std::vector< body_type >    m_bodies;
  simulator_type              m_simulator;
  configuration_type          m_configuration;
  material_library_type       m_library;

  material_type               m_material1;
  material_type               m_material2;

  box_type                    m_box;
  box_type                    m_big_box;
  box_type                    m_small_box;
  sphere_type                 m_sphere[3];
  mesh_type                   m_diku_mesh;
  mesh_type                   m_cow_mesh;
  mesh_type                   m_support_box_mesh;
  sdf_geometry_type           m_support_box_geo;
  sdf_geometry_type           m_cow_geo;
  sdf_geometry_type           m_diku_geo;


  // Used for simple joint scenes

  box_type               m_boxes[10];
  socket_type            m_socket_A;
  socket_type            m_socket_B;
  hinge_type             m_hinge;
  slider_type            m_slider;
  ball_type              m_ball;
  universal_type         m_universal;
  wheel_type             m_wheel;
  angular_limit_type     m_angular_limit1;
  angular_limit_type     m_angular_limit2;
  angular_motor_type     m_angular_motor1;
  angular_motor_type     m_angular_motor2;
  linear_limit_type      m_linear_limit;
  linear_motor_type      m_linear_motor;
  reach_cone_type        m_reach_cone;


};



// DATA_H
#endif
