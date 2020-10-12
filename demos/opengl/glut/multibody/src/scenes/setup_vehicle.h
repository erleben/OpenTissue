#ifndef SETUP_VEHICLE_H
#define SETUP_VEHICLE_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include "../data.h"

class Vehicle
{
public:

  box_type               m_ground_geometry;
  box_type               m_chasis_geometry;
  sphere_type            m_wheel_geometry;    // Sphere are really bad! Should perhaps be cylinders or capsules?

  body_type              m_ground;
  body_type              m_chasis;
  body_type              m_wheels[4];

  socket_type            m_wheel_sockets[4];
  socket_type            m_chasis_sockets[4];
  wheel_type             m_wheel_joints[4];

  angular_limit_type     m_wheel_limits[4];
  angular_motor_type     m_steering_motors[4];
  angular_motor_type     m_motors[4];

  real_type              m_speed;
  real_type              m_steer;

  material_type          wheel_ground_material;

public:

  void setup(Data & data)
  {
    size_type ground_material_idx = 1;
    size_type wheel_material_idx  = 2;

    m_speed = value_traits::zero();
    m_steer = value_traits::zero();

    // This method sets up a fairly simple box vehicle with spherical wheels!
    //
    // This kind of vehicle is not easy to control because we use isotropic friction and wheels are modelled as spheres!
    //
    // Spheres should be replaced with another geometry type and we should add anisotropic friction!
    //

    real_type const friction    = 0.25;
    real_type const restitution = 0.15;
    real_type const timestep    = 0.01;

    data.m_configuration.clear();
    data.m_library.clear();
    data.m_simulator.clear();

    quaternion_type Q;
    matrix3x3_type R;
    R = OpenTissue::math::diag(value_traits::one());

    m_ground_geometry.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(30,0.5,30));
    m_ground.set_position(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_ground.set_orientation(quaternion_type(1,0,0,0));
    m_ground.set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_ground.set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_ground.set_geometry(&m_ground_geometry);
    m_ground.set_fixed(true);
    m_ground.set_material_idx(ground_material_idx);
    data.m_configuration.add(&m_ground);

    m_chasis_geometry.set(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()),R,vector3_type(1, .5, 1.5));
    m_chasis.attach(&data.m_gravity);
    m_chasis.set_position(vector3_type(0,1.5,0));
    m_chasis.set_orientation(quaternion_type(1,0,0,0));
    m_chasis.set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_chasis.set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_chasis.set_geometry(&m_chasis_geometry);
    m_chasis.set_fixed(false);
    data.m_configuration.add(&m_chasis);

    m_wheel_geometry.radius(0.5);
    m_wheels[0].attach(&data.m_gravity);
    m_wheels[0].set_position(vector3_type(-1.5,value_traits::one(), -value_traits::one()));
    m_wheels[0].set_orientation(quaternion_type(1,0,0,0));
    m_wheels[0].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[0].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[0].set_geometry(&m_wheel_geometry);
    m_wheels[0].set_fixed(false);
    m_wheels[0].set_material_idx(wheel_material_idx);
    data.m_configuration.add(&m_wheels[0]);

    m_wheels[1].attach(&data.m_gravity);
    m_wheels[1].set_position(vector3_type( 1.5,value_traits::one(), -value_traits::one()));
    m_wheels[1].set_orientation(quaternion_type(1,0,0,0));
    m_wheels[1].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[1].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[1].set_geometry(&m_wheel_geometry);
    m_wheels[1].set_fixed(false);
    m_wheels[1].set_material_idx(wheel_material_idx);
    data.m_configuration.add(&m_wheels[1]);

    m_wheels[2].attach(&data.m_gravity);
    m_wheels[2].set_position(vector3_type(-1.5,value_traits::one(), value_traits::one()));
    m_wheels[2].set_orientation(quaternion_type(1,0,0,0));
    m_wheels[2].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[2].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[2].set_geometry(&m_wheel_geometry);
    m_wheels[2].set_fixed(false);
    m_wheels[2].set_material_idx(wheel_material_idx);
    data.m_configuration.add(&m_wheels[2]);

    m_wheels[3].attach(&data.m_gravity);
    m_wheels[3].set_position(vector3_type( 1.5,value_traits::one(), value_traits::one()));
    m_wheels[3].set_orientation(quaternion_type(1,0,0,0));
    m_wheels[3].set_velocity(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[3].set_spin(vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero()));
    m_wheels[3].set_geometry(&m_wheel_geometry);
    m_wheels[3].set_fixed(false);
    m_wheels[3].set_material_idx(wheel_material_idx);
    data.m_configuration.add(&m_wheels[3]);

    Q.Ry( value_traits::pi()/value_traits::two());
    m_wheel_sockets[0].init(m_wheels[0],coordsys_type(vector3_type(  .5,0,0),Q));  //--- right front
    m_wheel_sockets[1].init(m_wheels[1],coordsys_type(vector3_type( -.5,0,0),Q));  //--- left front
    m_wheel_sockets[2].init(m_wheels[2],coordsys_type(vector3_type(  .5,0,0),Q));  //--- right back
    m_wheel_sockets[3].init(m_wheels[3],coordsys_type(vector3_type( -.5,0,0),Q));  //--- left back

    Q.Rx( -value_traits::pi()/value_traits::two());
    m_chasis_sockets[0].init(m_chasis,coordsys_type(vector3_type( -1,-.5,-1),Q));  //--- right front
    m_chasis_sockets[1].init(m_chasis,coordsys_type(vector3_type(  1,-.5,-1),Q));  //--- left front
    m_chasis_sockets[2].init(m_chasis,coordsys_type(vector3_type( -1,-.5, 1),Q));  //--- right back
    m_chasis_sockets[3].init(m_chasis,coordsys_type(vector3_type(  1,-.5, 1),Q));  //--- left back

    for(size_type i=0;i<4;++i)
    {
      m_wheel_joints[i].set_frames_per_second(value_traits::one()/timestep);
      m_wheel_joints[i].set_error_reduction_parameter(0.8);
      m_wheel_joints[i].connect(m_chasis_sockets[i], m_wheel_sockets[i]);
      m_wheel_joints[i].set_suspension(0.8,0.4);

      m_motors[i].set_frames_per_second(value_traits::one()/timestep);
      m_motors[i].set_error_reduction_parameter(0.8);
      m_motors[i].set_maximum_force(0.1);
      m_motors[i].set_desired_speed(value_traits::zero());

      m_wheel_joints[i].set_wheel_motor(m_motors[i]);

      m_wheel_limits[i].set_frames_per_second(value_traits::one()/timestep);
      m_wheel_limits[i].set_error_reduction_parameter(0.8);
      m_wheel_limits[i].set_min_limit( value_traits::zero() );
      m_wheel_limits[i].set_max_limit( value_traits::zero() );

      m_wheel_joints[i].set_steering_limit( m_wheel_limits[i]);

      m_steering_motors[i].set_frames_per_second(value_traits::one()/timestep);
      m_steering_motors[i].set_error_reduction_parameter(0.8);
      m_steering_motors[i].set_maximum_force(value_traits::zero());
      m_steering_motors[i].set_desired_speed(value_traits::zero());
      m_wheel_joints[i].set_steering_motor(m_steering_motors[i]);

      data.m_configuration.add(&m_wheel_joints[i]);
    }

    data.m_gravity.set_acceleration(vector3_type(0,-9.81,0));

    data.m_simulator.init(data.m_configuration);

    // Setup default material properties
    material_type * default_material = data.m_library.default_material();
    default_material->set_friction_coefficient(friction);
    default_material->normal_restitution() = (restitution);

    // Setup material properties between wheels and ground
    wheel_ground_material.set_number_of_friction_directions(2);
    wheel_ground_material.set_friction_coefficient(0,2.0);
    wheel_ground_material.set_friction_coefficient(1,5.0);
    wheel_ground_material.set_material_indices(wheel_material_idx, ground_material_idx);
    wheel_ground_material.set_use_prefixed_direction(true);
    wheel_ground_material.set_prefixed_material_index( wheel_material_idx );
    wheel_ground_material.set_use_sliding_direction(false);
    wheel_ground_material.set_prefixed_direction( vector3_type( 0.0, 0.0, 1.0 ) );
    data.m_library.add( wheel_ground_material );

    // Add material library to the configuration
    data.m_configuration.set_material_library(data.m_library);

    data.m_simulator.get_stepper()->get_solver()->set_max_iterations(30);
    data.m_simulator.get_stepper()->warm_starting()      = false;
    data.m_simulator.get_stepper()->use_stabilization()  = true;
    data.m_simulator.get_stepper()->use_friction()       = true;
    data.m_simulator.get_stepper()->use_bounce()         = true;
  }

  void update()
  {
    using std::min;
    using std::max;

    //--- motor
    m_speed = min( 10.0,m_speed);
    m_speed = max(-10.0,m_speed);
    for(size_type i=0;i<4;++i)
    {
      m_motors[i].set_desired_speed(m_speed);
      m_motors[i].set_maximum_force(5.0);
    }
    //--- steering
    m_steer = min( value_traits::one(),m_steer);
    m_steer = max(-value_traits::one(),m_steer);
    for(size_type i=0;i<2;++i)
    {
      m_steering_motors[i].set_desired_speed(m_steer);
      m_steering_motors[i].set_maximum_force(5.0);
      m_wheel_limits[i].set_min_limit( -value_traits::pi()/4.0);
      m_wheel_limits[i].set_max_limit(  value_traits::pi()/4.0);
    }
    for(size_type i=0;i<4;++i)
    {
      if(m_wheel_joints[i].get_wheel_body())
      {
        m_wheel_joints[i].get_wheel_body()->set_finite_rotation_axis(m_wheel_joints[i].get_motor_axis_world());
      }
    }
  }

  void accelerate()  { m_speed -= 0.5;  } 

  void deccelerate() { m_speed += 0.5;  } 

  void turn_right()  { m_steer += 0.25; } 

  void turn_left()   { m_steer -= 0.25; } 

  void reset() {m_speed = 0;    m_steer = 0; }

  void explode()
  {
    vector3_type r;
    quaternion_type Q;

    OpenTissue::math::random(r,-value_traits::two(),value_traits::two());
    r(1) += 7;
    Q.random();
    Q = OpenTissue::math::normalize(Q);

    m_chasis.set_position(r);
    m_chasis.set_orientation(Q);

    for(size_type i=0;i<4;++i)
    {
      OpenTissue::math::random(r,-value_traits::two(),value_traits::two());
      r(1) += 4;
      Q.random();
      Q = OpenTissue::math::normalize(Q);
      m_wheels[i].set_position(r);
      m_wheels[i].set_orientation(Q);
    }
  }

};

// SETUP_VEHICLE_H
#endif




