#ifndef	OPENTISSUE_DYNAMICS_MBD_H
#define	OPENTISSUE_DYNAMICS_MBD_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>


//.. refactor this >>>>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_sweep_and_prune.h>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_spatial_hashing.h>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_exhaustive_search.h>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_geometry_dispatcher.h>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_setup_default_geometry_dispatcher.h>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_caching_contact_graph_analysis.h>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_single_group_analysis.h>
#include <OpenTissue/dynamics/mbd/collision_detection/mbd_collision_detection.h>
// <<<<<<

#include <OpenTissue/dynamics/mbd/joints/mbd_hinge_joint.h>
#include <OpenTissue/dynamics/mbd/joints/mbd_slider_joint.h>
#include <OpenTissue/dynamics/mbd/joints/mbd_ball_joint.h>
#include <OpenTissue/dynamics/mbd/joints/mbd_universal_joint.h>
#include <OpenTissue/dynamics/mbd/joints/mbd_wheel_joint.h>

#include <OpenTissue/dynamics/mbd/limits/mbd_reach_cone.h>
#include <OpenTissue/dynamics/mbd/limits/mbd_linear_joint_limit.h>
#include <OpenTissue/dynamics/mbd/motors/mbd_linear_joint_motor.h>
#include <OpenTissue/dynamics/mbd/limits/mbd_angular_joint_limit.h>
#include <OpenTissue/dynamics/mbd/motors/mbd_angular_joint_motor.h>

#include <OpenTissue/dynamics/mbd/scripted/mbd_oscillation.h>

#include <OpenTissue/dynamics/mbd/forces/mbd_gravity.h>
#include <OpenTissue/dynamics/mbd/forces/mbd_damping.h>
#include <OpenTissue/dynamics/mbd/forces/mbd_driving_force.h>

#include <OpenTissue/dynamics/mbd/solvers/mbd_projected_gauss_seidel.h>

#include <OpenTissue/dynamics/mbd/collision_resolvers/mbd_iterate_once_collision_resolver.h>
#include <OpenTissue/dynamics/mbd/collision_resolvers/mbd_sequential_collision_resolver.h>
#include <OpenTissue/dynamics/mbd/collision_resolvers/mbd_sequential_truncating_collision_resolver.h>

#include <OpenTissue/dynamics/mbd/steppers/mbd_2pass_shock_propagation_stepper.h>
#include <OpenTissue/dynamics/mbd/steppers/mbd_constraint_based_shock_propagation_stepper.h>
#include <OpenTissue/dynamics/mbd/steppers/mbd_dynamics_projection_stepper.h>
#include <OpenTissue/dynamics/mbd/steppers/mbd_dynamics_stepper.h>
#include <OpenTissue/dynamics/mbd/steppers/mbd_first_order_stepper.h>

#include <OpenTissue/dynamics/mbd/simulators/mbd_bisection_step_simulator.h>
#include <OpenTissue/dynamics/mbd/simulators/mbd_explicit_fixed_step_simulator.h>
#include <OpenTissue/dynamics/mbd/simulators/mbd_explicit_separate_error_correction_fixed_step_simulator.h>
#include <OpenTissue/dynamics/mbd/simulators/mbd_fix_point_step_simulator.h>
#include <OpenTissue/dynamics/mbd/simulators/mbd_implicit_fixed_step_simulator.h>
#include <OpenTissue/dynamics/mbd/simulators/mbd_semi_implicit_fixed_step_simulator.h>
#include <OpenTissue/dynamics/mbd/simulators/mbd_separated_collision_contact_fixed_step_simulator.h>

#include <OpenTissue/dynamics/mbd/collision_laws/mbd_newton_collision_law_policy.h>
#include <OpenTissue/dynamics/mbd/collision_laws/mbd_frictional_newton_collision_law_policy.h>
#include <OpenTissue/dynamics/mbd/collision_laws/mbd_chatterjee_ruina_collision_law_policy.h>


#include <OpenTissue/dynamics/mbd/mbd_compute_position_update.h>
#include <OpenTissue/dynamics/mbd/mbd_set_velocity_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_set_position_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_ncp_formulation.h>
#include <OpenTissue/dynamics/mbd/mbd_get_inverse_mass_matrix.h>
#include <OpenTissue/dynamics/mbd/mbd_get_mass_matrix.h>
#include <OpenTissue/dynamics/mbd/mbd_get_external_force_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_position_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_velocity_vector.h>

#include <OpenTissue/dynamics/mbd/mbd_compute_scripted_motions.h>
#include <OpenTissue/dynamics/mbd/mbd_is_all_bodies_sleepy.h>
#include <OpenTissue/dynamics/mbd/mbd_apply_impulse.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_collision_matrix.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_relative_contact_velocity.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_min_max_distance.h>


#include <OpenTissue/dynamics/mbd/mbd_compute_contact_count.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_contact_count_matrix.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_kinetic_energy_vector.h>

#include <OpenTissue/dynamics/mbd/mbd_kinetic_energy_sleepy_policy.h>
#include <OpenTissue/dynamics/mbd/mbd_no_sleepy_policy.h>

#include <OpenTissue/dynamics/mbd/mbd_types.h>

//--- Yrgk this really do not belong in a simulator engine...
#include <OpenTissue/dynamics/mbd/util/mbd_mel_geometry_string.h>
#include <OpenTissue/dynamics/mbd/util/mbd_mel_keyframe_string.h>
#include <OpenTissue/dynamics/mbd/util/mbd_mel_euler_filter_string.h>

//----------------------------- end yrgk!


// OPENTISSUE_DYNAMICS_MBD_H
#endif
