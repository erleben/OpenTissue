#ifndef TYPES_H
#define TYPES_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/dynamics/mbd/math/mbd_default_math_policy.h>
#include <OpenTissue/dynamics/mbd/math/mbd_optimized_ublas_math_policy.h>
#include <OpenTissue/dynamics/mbd/mbd.h>

template<typename types>
class MyCollisionDetection
  : public OpenTissue::mbd::CollisionDetection<
  types
  , OpenTissue::mbd::SpatialHashing
  , OpenTissue::mbd::GeometryDispatcher
  , OpenTissue::mbd::SingleGroupAnalysis
  >
{};

template< typename types  >
class MyStepper
  : public OpenTissue::mbd::DynamicsStepper< 
  types  
  , OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy >  
  >
{};

typedef OpenTissue::mbd::Types<
OpenTissue::mbd::optimized_ublas_math_policy<double>
, OpenTissue::mbd::NoSleepyPolicy
, MyStepper
, MyCollisionDetection
, OpenTissue::mbd::ExplicitFixedStepSimulator
> retro_types;

typedef retro_types::simulator_type                          simulator_type;
typedef retro_types::body_type                               body_type;
typedef retro_types::socket_type                             socket_type;
typedef retro_types::configuration_type                      configuration_type;
typedef retro_types::material_library_type                   material_library_type;
typedef retro_types::material_type                           material_type;

typedef retro_types::math_policy                             math_policy;
typedef retro_types::math_policy::index_type                 size_type;
typedef retro_types::math_policy::real_type                  real_type;
typedef retro_types::math_policy::vector3_type               vector3_type;
typedef retro_types::math_policy::quaternion_type            quaternion_type;
typedef retro_types::math_policy::matrix3x3_type             matrix3x3_type;
typedef retro_types::math_policy::vector_type                vector_type;
typedef retro_types::math_policy::matrix_type                matrix_type;
typedef retro_types::math_policy::value_traits               value_traits;
typedef retro_types::math_policy::coordsys_type              coordsys_type;

typedef OpenTissue::mbd::Gravity<retro_types>         gravity_type;
typedef OpenTissue::mbd::Damping<retro_types>         damping_type;

typedef retro_types::node_traits                        node_traits;
typedef OpenTissue::geometry::Sphere<math_policy>       sphere_type;
typedef OpenTissue::geometry::Plane<math_policy>        plane_type;
typedef OpenTissue::geometry::OBB<math_policy>          box_type;
typedef OpenTissue::polymesh::PolyMesh<math_policy>     mesh_type;
typedef OpenTissue::grid::Grid<float,math_policy>       grid_type;
typedef OpenTissue::sdf::Geometry<mesh_type,grid_type>  sdf_geometry_type;

typedef OpenTissue::mbd::WheelJoint<retro_types>           wheel_type;
typedef OpenTissue::mbd::BallJoint<retro_types>            ball_type;
typedef OpenTissue::mbd::UniversalJoint<retro_types>       universal_type;
typedef OpenTissue::mbd::HingeJoint<retro_types>           hinge_type;
typedef OpenTissue::mbd::SliderJoint<retro_types>          slider_type;
typedef OpenTissue::mbd::AngularJointMotor<retro_types>    angular_motor_type;
typedef OpenTissue::mbd::AngularJointLimit<retro_types>    angular_limit_type;
typedef OpenTissue::mbd::LinearJointMotor<retro_types>     linear_motor_type;
typedef OpenTissue::mbd::LinearJointLimit<retro_types>     linear_limit_type;
typedef OpenTissue::mbd::ReachCone<retro_types>            reach_cone_type;


// TYPES_H
#endif
