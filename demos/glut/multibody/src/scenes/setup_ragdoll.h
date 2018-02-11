#ifndef SETUP_RAGDOLL_H
#define SETUP_RAGDOLL_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_compute_sphere_mass_properties.h>

#include "../data.h"

class Ragdoll
{
public:

protected:

  box_type            m_boxes[21];
  ball_type           m_balls[6];
  socket_type         m_sockets[30];
  hinge_type          m_hinges[9];
  angular_limit_type  m_angular_limits[9];

  ball_type           m_locks[4];
  socket_type         m_anchors[8];
  sphere_type         m_rock;


public:

  Ragdoll(){}

protected:

  void setup_hinge_joint (
    body_type *A
    , body_type *B
    , vector3_type const & location
    , quaternion_type const & hinge_axis
    , real_type const & min_value
    , real_type const & max_value
    , socket_type * socket_A
    , socket_type * socket_B
    , hinge_type * hinge
    , angular_limit_type * angular_limit
    , real_type const & timestep
    )
  {
    vector3_type position_A;
    vector3_type position_B;

    A->get_position( position_A );
    B->get_position( position_B );

    // Socket_A 's quarternion decides which axis the hinge evolves around .
    socket_A->init(*A, coordsys_type( location - position_A , hinge_axis ) );
    socket_B->init(*B, coordsys_type( location - position_B , hinge_axis ) );
    hinge->connect(*socket_A , *socket_B );

    angular_limit->set_min_limit( min_value );
    angular_limit->set_max_limit( max_value );

    hinge->set_limit( *angular_limit );
    hinge->set_frames_per_second( 1.0/ timestep );
    hinge->set_error_reduction_parameter( 0.8 );
  }

  void setup_ball_joint( 
    body_type *A
    , body_type *B
    , vector3_type const & location 
    , socket_type * socket_A
    , socket_type * socket_B
    , ball_type * ball
    , real_type const & timestep
    )
  {
    vector3_type position_A;
    vector3_type position_B;

    A-> get_position( position_A );
    B-> get_position( position_B );

    socket_A-> init( *A, coordsys_type( location - position_A , quaternion_type ()) );
    socket_B-> init( *B, coordsys_type( location - position_B , quaternion_type ()) );
    ball->connect( *socket_A, *socket_B );

    ball->set_frames_per_second( 1.0/ timestep );
    ball->set_error_reduction_parameter( 0.8) ;
  }

public:

  void setup(Data & data )
  {
    vector3_type const position    = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one()*10);
    real_type    const size        = value_traits::one()/10;
    vector3_type const zero        = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
    real_type    const friction    = 0.25;
    real_type    const restitution = 0.15;
    real_type    const timestep    = 0.01;

    data.m_configuration.clear();
    data.m_library.clear();
    data.m_simulator.clear();

    data.m_bodies.clear();
    data.m_bodies.resize(22);

    // Setup geometries
    {
      matrix3x3_type R;
      R = OpenTissue::math::diag( value_traits::one() ) ;
      m_boxes[0].set( zero ,R, vector3_type(1000 ,1000 ,10) /2*size ); // Ground
      m_boxes[1].set( zero ,R, vector3_type(15.7 ,20 ,20)/2* size ); //Head
      m_boxes[2].set( zero ,R, vector3_type(10.9 ,10.9 ,12.3)/2* size ); // Neck
      m_boxes[3].set( zero ,R, vector3_type(39.2 ,25 ,40.9)/2* size );// Chest
      m_boxes[4].set( zero ,R, vector3_type(35.8 ,14.3 ,14.5)/2* size ); // Hip
      m_boxes[5].set( zero ,R, vector3_type(36.6 ,8.8 ,8.8)/2* size );// Overarm
      m_boxes[6].set( zero ,R, vector3_type(28.8 ,8.5 ,8.5)/2* size );// Underarm
      m_boxes[7].set( zero ,R, vector3_type(8.9 ,8.9 ,8.9)/2* size );// Hand
      m_boxes[8].set( zero ,R, vector3_type(16.9 ,16.9 ,51.2)/2* size ); // Thigh
      m_boxes[9].set( zero ,R, vector3_type(10.6 ,10.6 ,30.5)/2* size ); // Calf
      m_boxes[10].set( zero ,R, vector3_type(9.9 ,27.3 ,13.9)/2* size ); // Foot
    }

    quaternion_type Q;
    Q.identity();

    // ground
    data.m_bodies[0].set_position( vector3_type (0 ,0 , -250)* size + position );
    data.m_bodies[0].set_orientation( Q );
    data.m_bodies[0].set_velocity( zero);
    data.m_bodies[0].set_spin( zero);
    data.m_bodies[0].set_geometry( &m_boxes[0]) ;
    data.m_bodies[0].set_fixed( true );
    data.m_configuration.add( &data.m_bodies[0] );

    // head
    data.m_bodies[1].attach( &data.m_gravity );
    data.m_bodies[1].set_position( zero*size + position );
    data.m_bodies[1].set_orientation( Q );
    data.m_bodies[1].set_velocity( zero );
    data.m_bodies[1].set_spin( zero );
    data.m_bodies[1].set_geometry( &m_boxes[1] ) ;
    data.m_bodies[1].set_fixed ( false );
    data.m_configuration.add( &data.m_bodies[1] );

    // neck
    data.m_bodies[2].attach( &data.m_gravity );
    data.m_bodies[2].set_position( vector3_type (0 ,0 , -16.15) * size + position );
    data.m_bodies[2].set_orientation( Q );
    data.m_bodies[2].set_velocity( zero );
    data.m_bodies[2].set_spin( zero );
    data.m_bodies[2].set_geometry( &m_boxes [2]) ;
    data.m_bodies[2].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[2] );

    // chest
    data.m_bodies[3].attach( &data.m_gravity );
    data.m_bodies[3].set_position( vector3_type (0 ,0 , -42.75) * size + position );
    data.m_bodies[3].set_orientation( Q );
    data.m_bodies[3].set_velocity( zero );
    data.m_bodies[3].set_spin( zero );
    data.m_bodies[3].set_geometry( &m_boxes [3] ) ;
    data.m_bodies[3].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[3] );

    // hip
    data.m_bodies[4].attach( &data.m_gravity );
    data.m_bodies[4].set_position( vector3_type (0 ,0 , -70.45)*size + position );
    data.m_bodies[4].set_orientation( Q );
    data.m_bodies[4].set_velocity( zero );
    data.m_bodies[4].set_spin( zero );
    data.m_bodies[4].set_geometry( &m_boxes[4] ) ;
    data.m_bodies[4].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[4] );

    // left overarm
    data.m_bodies[5].attach( &data.m_gravity );
    data.m_bodies[5].set_position( vector3_type (37.9 ,0 , -26.7) * size + position );
    data.m_bodies[5].set_orientation( Q );
    data.m_bodies[5].set_velocity( zero );
    data.m_bodies[5].set_spin( zero );
    data.m_bodies[5].set_geometry( &m_boxes[5] ) ;
    data.m_bodies[5].set_fixed ( false );
    data.m_configuration.add( &data.m_bodies[5] );

    // right overarm
    data.m_bodies[6].attach( &data.m_gravity );
    data.m_bodies[6].set_position( vector3_type ( -37.9 ,0 , -26.7) * size + position );
    data.m_bodies[6].set_orientation( Q );
    data.m_bodies[6].set_velocity( zero );
    data.m_bodies[6].set_spin( zero );
    data.m_bodies[6].set_geometry( &m_boxes[5]) ;
    data.m_bodies[6].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[6] );

    // left underarm
    data.m_bodies[7].attach( &data.m_gravity );
    data.m_bodies[7].set_position( vector3_type (70.6 ,0 , -26.7) * size + position );
    data.m_bodies[7].set_orientation( Q );
    data.m_bodies[7].set_velocity( zero );
    data.m_bodies[7].set_spin( zero );
    data.m_bodies[7].set_geometry( &m_boxes[6] );
    data.m_bodies[7].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[7] );

    // right underarm
    data.m_bodies[8].attach( &data.m_gravity );
    data.m_bodies[8].set_position( vector3_type ( -70.6 ,0 , -26.7) * size + position );
    data.m_bodies[8].set_orientation( Q );
    data.m_bodies[8].set_velocity( zero );
    data.m_bodies[8].set_spin( zero );
    data.m_bodies[8].set_geometry( &m_boxes[6] );
    data.m_bodies[8].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[8] );

    // left hand
    data.m_bodies[9].attach( &data.m_gravity );
    data.m_bodies[9].set_position( vector3_type (89.45 ,0 , -26.7) * size + position );
    data.m_bodies[9].set_orientation( Q );
    data.m_bodies[9].set_velocity( zero );
    data.m_bodies[9].set_spin( zero );
    data.m_bodies[9].set_geometry( &m_boxes[7] );
    data.m_bodies[9].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[9] );

    // right hand
    data.m_bodies[10].attach( &data.m_gravity );
    data.m_bodies[10].set_position( vector3_type ( -89.45 ,0 , -26.7) * size + position );
    data.m_bodies[10].set_orientation( Q );
    data.m_bodies[10].set_velocity( zero );
    data.m_bodies[10].set_spin( zero );
    data.m_bodies[10].set_geometry( &m_boxes[7] );
    data.m_bodies[10].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[10] );

    // left thigh
    data.m_bodies[11].attach( &data.m_gravity );
    data.m_bodies[11].set_position( vector3_type (9.45 ,0 , -103.3) * size + position );
    data.m_bodies[11].set_orientation( Q );
    data.m_bodies[11].set_velocity( zero );
    data.m_bodies[11].set_spin( zero );
    data.m_bodies[11].set_geometry( &m_boxes[8] );
    data.m_bodies[11].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[11] );

    // right thigh
    data.m_bodies[12].attach( &data.m_gravity );
    data.m_bodies[12].set_position( vector3_type ( -9.45 ,0 , -103.3) * size + position );
    data.m_bodies[12].set_orientation( Q );
    data.m_bodies[12].set_velocity( zero );
    data.m_bodies[12].set_spin( zero );
    data.m_bodies[12].set_geometry( &m_boxes[8] );
    data.m_bodies[12].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[12] );

    // left calf
    data.m_bodies[13].attach( &data.m_gravity );
    data.m_bodies[13].set_position( vector3_type (9.45 ,0 , -144.15) * size + position );
    data.m_bodies[13].set_orientation( Q );
    data.m_bodies[13].set_velocity( zero );
    data.m_bodies[13].set_spin( zero );
    data.m_bodies[13].set_geometry( &m_boxes[9] );
    data.m_bodies[13].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[13] );

    // right calf
    data.m_bodies[14].attach( &data.m_gravity );
    data.m_bodies[14].set_position( vector3_type ( -9.45 ,0 , -144.15) * size + position );
    data.m_bodies[14].set_orientation( Q );
    data.m_bodies[14].set_velocity( zero );
    data.m_bodies[14].set_spin( zero );
    data.m_bodies[14].set_geometry( &m_boxes[9] );
    data.m_bodies[14].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[14] );

    // left foot
    data.m_bodies[15].attach ( &data.m_gravity );
    data.m_bodies[15].set_position( vector3_type (9.45 , -5.2 , -166.45) * size + position );
    data.m_bodies[15].set_orientation( Q );
    data.m_bodies[15].set_velocity( zero );
    data.m_bodies[15].set_spin( zero );
    data.m_bodies[15].set_geometry( &m_boxes[10] );
    data.m_bodies[15].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[15] );

    // right foot
    data.m_bodies[16].attach( &data.m_gravity );
    data.m_bodies[16].set_position( vector3_type ( -9.45 , -5.2 , -166.45) * size + position );
    data.m_bodies[16].set_orientation( Q );
    data.m_bodies[16].set_velocity( zero );
    data.m_bodies[16].set_spin( zero );
    data.m_bodies[16].set_geometry( &m_boxes[10] ) ;
    data.m_bodies[16].set_fixed( false );
    data.m_configuration.add( &data.m_bodies[16] );

    quaternion_type x, y, z;
    x = quaternion_type (1 ,1 ,1 ,1);
    y = quaternion_type (0 ,0 ,1 ,1);
    z = quaternion_type (1 ,0 ,0 ,0);

    // attach head to neck
    {
      real_type       min_value = -0.25*value_traits::pi();
      real_type       max_value =  0.25*value_traits::pi();
      vector3_type    loc       = vector3_type (0 ,0 , -10)* size + position;
      quaternion_type axis      = x;
      setup_hinge_joint( &data.m_bodies[1], &data.m_bodies[2], loc, axis, min_value, max_value
        , &m_sockets[0], &m_sockets[1], &m_hinges[0], &m_angular_limits[0], timestep);
      data.m_configuration.add( &m_hinges[0] );
    }
    // attach left underarm to left overarm
    {
      real_type       min_value = -0.75*value_traits::pi();
      real_type       max_value =  value_traits::zero();
      vector3_type    loc       = vector3_type (52.3 ,0 , -26.7) * size + position;
      quaternion_type axis      = z;
      setup_hinge_joint( &data.m_bodies[7], &data.m_bodies[5], loc, axis, min_value, max_value
        , &m_sockets[2], &m_sockets[3], &m_hinges[1], &m_angular_limits[1], timestep );
      data.m_configuration.add( &m_hinges[1] );
    }
    // attach right underarm right overarm
    {
      real_type       min_value = -0.75*value_traits::pi();
      real_type       max_value =  value_traits::zero();
      vector3_type    loc       = vector3_type ( -52.3 ,0 , -26.7) * size + position;
      quaternion_type axis      = z;
      setup_hinge_joint( &data.m_bodies[8], &data.m_bodies[6], loc, axis, min_value, max_value
        , &m_sockets[4], &m_sockets[5], &m_hinges[2], &m_angular_limits[2], timestep );
      data.m_configuration.add( &m_hinges[2] );
    }
    // attach left hand to left underarm
    {
      real_type       min_value = -value_traits::pi_half();
      real_type       max_value =  value_traits::pi_half();
      vector3_type    loc       = vector3_type (81.1 ,0 , -26.7) * size + position;
      quaternion_type axis      = x;
      setup_hinge_joint( &data.m_bodies[9], &data.m_bodies[7], loc, axis, min_value, max_value
        , &m_sockets[6], &m_sockets[7], &m_hinges[3], &m_angular_limits[3], timestep );
      data.m_configuration.add( &m_hinges[3] );
    }
    // attach right hand to right underarm
    {
      real_type       min_value = -value_traits::pi_half();
      real_type       max_value =  value_traits::pi_half();
      vector3_type    loc       = vector3_type( -81.1 ,0 , -26.7) * size + position;
      quaternion_type axis      = x;
      setup_hinge_joint( &data.m_bodies[10], &data.m_bodies[8], loc, axis, min_value, max_value
        , &m_sockets[8], &m_sockets[9], &m_hinges[4], &m_angular_limits[4], timestep );
      data.m_configuration.add( &m_hinges[4] );
    }
    // attach left calf to left thigh
    {
      real_type       min_value = -0.75*value_traits::pi();
      real_type       max_value =  value_traits::zero();
      vector3_type    loc       = vector3_type(9.45 ,0 , -128.9) * size + position;
      quaternion_type axis      = x;
      setup_hinge_joint( &data.m_bodies[13], &data.m_bodies[11], loc, axis, min_value, max_value
        , &m_sockets[10], &m_sockets[11], &m_hinges[5], &m_angular_limits[5], timestep );
      data.m_configuration.add( &m_hinges[5] );
    }
    // attach right calf to right thigh
    {
      real_type       min_value = -0.75*value_traits::pi();
      real_type       max_value =  value_traits::zero();
      vector3_type    loc       = vector3_type( -9.45 ,0 , -128.9) * size + position;
      quaternion_type axis      = x;
      setup_hinge_joint( &data.m_bodies[14], &data.m_bodies[12], loc, axis, min_value, max_value
        , &m_sockets[12], &m_sockets[13], &m_hinges[6], &m_angular_limits[6], timestep );
      data.m_configuration.add( &m_hinges[6] );
    }
    // attach left foot to left calf
    {
      real_type       min_value = -value_traits::pi_half();
      real_type       max_value =  value_traits::zero();
      vector3_type    loc       = vector3_type(9.45 ,0 , -159.4) * size + position;
      quaternion_type axis      = x;
      setup_hinge_joint( &data.m_bodies[15], &data.m_bodies[13], loc, axis, min_value, max_value
        , &m_sockets[14], &m_sockets[15], &m_hinges[7], &m_angular_limits[7], timestep );
      data.m_configuration.add( &m_hinges[7] );
    }
    // attach right foot to right calf
    {
      real_type       min_value = -value_traits::pi_half();
      real_type       max_value =  value_traits::zero();
      vector3_type    loc       = vector3_type( -9.45 ,0 , -159.4) * size + position;
      quaternion_type axis      = x;
      setup_hinge_joint( &data.m_bodies[16], &data.m_bodies[14], loc, axis, min_value, max_value
        , &m_sockets[16], &m_sockets[17], &m_hinges[8], &m_angular_limits[8], timestep );
      data.m_configuration.add( &m_hinges[8] );
    }
    // attach neck to chest
    {
      vector3_type loc = vector3_type (0 ,0 , -22.3)* size + position;
      setup_ball_joint( &data.m_bodies[2], &data.m_bodies[3], loc
        , &m_sockets[18], &m_sockets[19], &m_balls[0], timestep );
      data.m_configuration.add( &m_balls[0] );
    }
    // attach hip to chest
    {
      vector3_type loc = vector3_type (0 ,0 , -61.5)* size + position;
      setup_ball_joint( &data.m_bodies[4], &data.m_bodies[3], loc
        , &m_sockets[20], &m_sockets[21], &m_balls[1], timestep );
      data.m_configuration.add( &m_balls[1] );
    }
    // attach left overarm to chest
    {
      vector3_type loc = vector3_type (19.6 ,0 , -26.7) * size + position;
      setup_ball_joint( &data.m_bodies[5], &data.m_bodies[3], loc
        , &m_sockets[22], &m_sockets[23], &m_balls[2], timestep );
      data.m_configuration.add( &m_balls[2] );
    }
    // attach right overarm to chest
    {
      vector3_type loc = vector3_type ( -19.6 ,0 , -26.7) * size + position;
      setup_ball_joint( &data.m_bodies[6], &data.m_bodies[3], loc
        , &m_sockets[24], &m_sockets[25], &m_balls[3], timestep );
      data.m_configuration.add( &m_balls[3] );
    }
    // attach left thigh to hip
    {
      vector3_type loc = vector3_type (9.45 ,0 , -77.7) * size + position ;
      setup_ball_joint( &data.m_bodies[11], &data.m_bodies[4], loc
        , &m_sockets[26], &m_sockets[27], &m_balls[4], timestep );
      data.m_configuration.add( &m_balls[4] );
    }
    // attach right thigh to hip
    {
      vector3_type loc = vector3_type ( -9.45 ,0 , -77.7) * size + position ;
      setup_ball_joint( &data.m_bodies[12], &data.m_bodies[4], loc
        , &m_sockets[28], &m_sockets[29], &m_balls[5], timestep );
      data.m_configuration.add( &m_balls[5] );
    }

    data.m_gravity.set_acceleration(vector3_type(0,0,-9.81));
    data.m_simulator.init(data.m_configuration);

    // Setup default material properties
    material_type * default_material = data.m_library.default_material();
    default_material->set_friction_coefficient(friction);
    default_material->normal_restitution() = (restitution);

    // Add material library to the configuration
    data.m_configuration.set_material_library(data.m_library);

    data.m_simulator.get_stepper()->get_solver()->set_max_iterations(100);
    data.m_simulator.get_stepper()->warm_starting()      = false;
    data.m_simulator.get_stepper()->use_stabilization()  = true;
    data.m_simulator.get_stepper()->use_friction()       = true;
    data.m_simulator.get_stepper()->use_bounce()         = true;
  }


  void torture1(Data & data )
  {
    vector3_type const position    = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one()*10);
    vector3_type const zero        = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
    real_type    const timestep    = 0.01;

    quaternion_type Q;
    Q.identity();

    this->setup(data);

    // Get geometry information about setup

    vector3_type left_hand_pos;
    vector3_type right_hand_pos;
    vector3_type left_foot_pos;
    vector3_type right_foot_pos;

    vector3_type hand_ext = m_boxes[7].ext();
    vector3_type foot_ext = m_boxes[10].ext();
    vector3_type pillar_ext = vector3_type( 1.0, 1.0, 20.0 );

    body_type * left_hand = &(data.m_bodies[9]);
    body_type * right_hand = &(data.m_bodies[10]);
    body_type * left_foot = &(data.m_bodies[15]);
    body_type * right_foot = &(data.m_bodies[16]);
    body_type * pillar1 = &(data.m_bodies[17]);
    body_type * pillar2 = &(data.m_bodies[18]);
    body_type * pillar3 = &(data.m_bodies[19]);
    body_type * pillar4 = &(data.m_bodies[20]);
    body_type * rock = &(data.m_bodies[21]);

    left_hand->get_position(left_hand_pos);
    right_hand->get_position(right_hand_pos);
    left_foot->get_position(left_foot_pos);
    right_foot->get_position(right_foot_pos);
    vector3_type pillar1_pos = position + vector3_type( 4.0,  10.0, -5.0);
    vector3_type pillar2_pos = position + vector3_type(-4.0,  10.0, -5.0);
    vector3_type pillar3_pos = position + vector3_type( 4.0, -10.0, -5.0);
    vector3_type pillar4_pos = position + vector3_type(-4.0, -10.0, -5.0);
    vector3_type rock_pos = vector3_type(0,0,30);

    // Setup geomtries
    matrix3x3_type R;
    R = OpenTissue::math::diag( value_traits::one() ) ;
    m_boxes[20].set( zero ,R, pillar_ext );
    m_rock.set( zero, 9.25);

    // Setup bodies
    pillar1->set_position( pillar1_pos );
    pillar1->set_orientation( Q );
    pillar1->set_velocity( zero);
    pillar1->set_spin( zero);
    pillar1->set_geometry( &m_boxes[20]) ;
    pillar1->set_fixed( true );
    data.m_configuration.add( pillar1 );

    pillar2->set_position( pillar2_pos );
    pillar2->set_orientation( Q );
    pillar2->set_velocity( zero);
    pillar2->set_spin( zero);
    pillar2->set_geometry( &m_boxes[20]) ;
    pillar2->set_fixed( true );
    data.m_configuration.add( pillar2 );

    pillar3->set_position( pillar3_pos );
    pillar3->set_orientation( Q );
    pillar3->set_velocity( zero);
    pillar3->set_spin( zero);
    pillar3->set_geometry( &m_boxes[20]) ;
    pillar3->set_fixed( true );
    data.m_configuration.add( pillar3 );

    pillar4->set_position( pillar4_pos );
    pillar4->set_orientation( Q );
    pillar4->set_velocity( zero);
    pillar4->set_spin( zero);
    pillar4->set_geometry( &m_boxes[20]) ;
    pillar4->set_fixed( true );
    data.m_configuration.add( pillar4 );

    rock->attach( &data.m_gravity );
    rock->set_position( rock_pos );
    rock->set_orientation( Q );
    rock->set_velocity( zero );
    rock->set_spin( zero );
    rock->set_geometry( &m_rock ) ;
    rock->set_fixed ( false );
    matrix3x3_type I;
    real_type density = 7200;
    real_type mass;
    vector3_type inertia_vec;
    OpenTissue::geometry::compute_sphere_mass_properties(m_rock.radius(),density,mass,inertia_vec);
    I= OpenTissue::math::diag(inertia_vec(0),inertia_vec(1),inertia_vec(2));
    rock->set_mass(mass);
    rock->set_inertia_bf(I);
    data.m_configuration.add( rock );

    // Setup locks 
    {
      vector3_type  pillar_location = vector3_type(-pillar_ext(0),-pillar_ext(1), 0.0);
      vector3_type  hand_location   = vector3_type(hand_ext(0),0.0, 0.0);     
      vector3_type axis = unit( vector3_type(1.0,-1.0,0.0) );
      real_type radian = value_traits::pi_half();
      Q.Ru( radian, axis );
      m_anchors[0].init( *pillar1, coordsys_type( pillar_location , Q ) );
      Q.Ry(-radian);
      m_anchors[1].init( *left_hand, coordsys_type( hand_location , Q ) );
      m_locks[0].connect( m_anchors[0], m_anchors[1] );
      m_locks[0].set_frames_per_second( 1.0/ timestep );
      m_locks[0].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[0] );
    }
    {
      vector3_type  pillar_location = vector3_type(pillar_ext(0),-pillar_ext(1), 0.0);
      vector3_type  hand_location   = vector3_type(-hand_ext(0),0.0, 0.0);     
      vector3_type axis = unit( vector3_type(1.0,1.0,0.0) );
      real_type radian = value_traits::pi_half();
      Q.Ru( radian, axis );
      m_anchors[2].init( *pillar2, coordsys_type( pillar_location , Q ) );
      Q.Ry(radian);
      m_anchors[3].init( *right_hand, coordsys_type( hand_location , Q ) );
      m_locks[1].connect( m_anchors[2], m_anchors[3] );
      m_locks[1].set_frames_per_second( 1.0/ timestep );
      m_locks[1].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[1] );
    }
    {
      vector3_type  pillar_location = vector3_type(-pillar_ext(0),pillar_ext(1), 0.0);
      vector3_type  foot_location   = vector3_type(0.0,-foot_ext(1), 0.0);     
      vector3_type axis = unit( vector3_type(-1.0,-1.0,0.0) );
      real_type radian = value_traits::pi_half();
      Q.Ru( radian, axis );
      m_anchors[4].init( *pillar3, coordsys_type( pillar_location , Q ) );
      Q.Rx(-radian);
      m_anchors[5].init( *left_foot, coordsys_type( foot_location , Q ) );
      m_locks[2].connect( m_anchors[4], m_anchors[5] );
      m_locks[2].set_frames_per_second( 1.0/ timestep );
      m_locks[2].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[2] );
    }
    {
      vector3_type  pillar_location = vector3_type(pillar_ext(0),pillar_ext(1), 0.0);
      vector3_type  foot_location   = vector3_type(0.0,-foot_ext(1), 0.0);     
      vector3_type axis = unit( vector3_type(-1.0, 1.0,0.0) );
      real_type radian = value_traits::pi_half();
      Q.Ru( radian, axis );
      m_anchors[6].init( *pillar4, coordsys_type( pillar_location , Q ) );
      Q.Rx(-radian);
      m_anchors[7].init( *right_foot, coordsys_type( foot_location , Q ) );
      m_locks[3].connect( m_anchors[6], m_anchors[7] );
      m_locks[3].set_frames_per_second( 1.0/ timestep );
      m_locks[3].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[3] );
    }

  }


  void torture2(Data & data )
  {
    vector3_type const position    = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one()*10);
    vector3_type const zero        = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
    real_type    const timestep    = 0.01;

    quaternion_type Q;
    Q.identity();

    this->setup(data);

    // Get geometry information about setup

    vector3_type left_hand_pos;
    vector3_type right_hand_pos;
    vector3_type left_foot_pos;
    vector3_type right_foot_pos;

    vector3_type hand_ext = m_boxes[7].ext();
    vector3_type foot_ext = m_boxes[10].ext();
    vector3_type pillar_ext = vector3_type( 20.0, 1.0, 1.0 );

    body_type * left_hand = &(data.m_bodies[9]);
    body_type * right_hand = &(data.m_bodies[10]);
    body_type * left_foot = &(data.m_bodies[15]);
    body_type * right_foot = &(data.m_bodies[16]);
    body_type * pillar = &(data.m_bodies[17]);
    body_type * rock = &(data.m_bodies[21]);

    left_hand->get_position(left_hand_pos);
    right_hand->get_position(right_hand_pos);
    left_foot->get_position(left_foot_pos);
    right_foot->get_position(right_foot_pos);
    vector3_type pillar_pos = position + vector3_type( 0.0,  0.0, 10.0);
    vector3_type rock_pos = vector3_type(0,0,-10);

    // Setup geomtries
    matrix3x3_type R;
    R = OpenTissue::math::diag( value_traits::one() ) ;
    m_boxes[20].set( zero ,R, pillar_ext );
    m_rock.set( zero, 3.00);

    // Setup bodies
    pillar->set_position( pillar_pos );
    pillar->set_orientation( Q );
    pillar->set_velocity( zero);
    pillar->set_spin( zero);
    pillar->set_geometry( &m_boxes[20]) ;
    pillar->set_fixed( true );
    data.m_configuration.add( pillar );

    rock->attach( &data.m_gravity );
    rock->set_position( rock_pos );
    rock->set_orientation( Q );
    rock->set_velocity( zero );
    rock->set_spin( zero );
    rock->set_geometry( &m_rock ) ;
    rock->set_fixed ( false );
    matrix3x3_type I;
    real_type density = 7200;
    real_type mass;
    vector3_type inertia_vec;
    OpenTissue::geometry::compute_sphere_mass_properties(m_rock.radius(),density,mass,inertia_vec);
    I= OpenTissue::math::diag(inertia_vec(0),inertia_vec(1),inertia_vec(2));
    rock->set_mass(mass);
    rock->set_inertia_bf(I);
    data.m_configuration.add( rock );

    // Setup locks 
    {
      vector3_type  pillar_location = vector3_type( 3.0, 0.0, -pillar_ext(2));
      vector3_type  hand_location   = vector3_type(hand_ext(0),0.0, 0.0);     

      Q.Rx(value_traits::pi());
      m_anchors[0].init( *pillar, coordsys_type( pillar_location , Q ) );
      Q.Ry(- value_traits::pi_half());
      m_anchors[1].init( *left_hand, coordsys_type( hand_location , Q ) );

      m_locks[0].connect( m_anchors[0], m_anchors[1] );
      m_locks[0].set_frames_per_second( 1.0/ timestep );
      m_locks[0].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[0] );
    }
    {
      vector3_type  pillar_location = vector3_type(-3.0, 0.0, -pillar_ext(2));
      vector3_type  hand_location   = vector3_type(-hand_ext(0),0.0, 0.0);     

      Q.Rx(value_traits::pi());
      m_anchors[2].init( *pillar, coordsys_type( pillar_location , Q ) );
      Q.Ry(value_traits::pi_half());
      m_anchors[3].init( *right_hand, coordsys_type( hand_location , Q ) );

      m_locks[1].connect( m_anchors[2], m_anchors[3] );
      m_locks[1].set_frames_per_second( 1.0/ timestep );
      m_locks[1].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[1] );
    }
    {
      vector3_type  rock_location = vector3_type(3.0,0.0,0.0);
      vector3_type  foot_location   = vector3_type(-foot_ext(0),0.0, 0.0);     

      Q.Ry(-value_traits::pi_half());
      m_anchors[4].init( *rock, coordsys_type( rock_location , Q ) );
      Q.Ry(-value_traits::pi_half());
      m_anchors[5].init( *left_foot, coordsys_type( foot_location , Q ) );

      m_locks[2].connect( m_anchors[4], m_anchors[5] );
      m_locks[2].set_frames_per_second( 1.0/ timestep );
      m_locks[2].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[2] );
    }
    {
      vector3_type  rock_location = vector3_type(-3.0,0.0,0.0);
      vector3_type  foot_location   = vector3_type(foot_ext(0),0.0, 0.0);

      Q.Ry(value_traits::pi_half());
      m_anchors[6].init( *rock, coordsys_type( rock_location , Q ) );
      Q.Ry(value_traits::pi_half());
      m_anchors[7].init( *right_foot, coordsys_type( foot_location , Q ) );

      m_locks[3].connect( m_anchors[6], m_anchors[7] );
      m_locks[3].set_frames_per_second( 1.0/ timestep );
      m_locks[3].set_error_reduction_parameter( 0.8) ;
      data.m_configuration.add( &m_locks[3] );
    }

  }

};

// SETUP_RAGDOLL_H
#endif
