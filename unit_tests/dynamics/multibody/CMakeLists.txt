#----------------------------------------------------------------------
#
# We got a bunch of compile tests, it really make no sense to build an
# executable from these. We just create a static lib for each of these.
#
set(SRC_FILES
  src/simulator_types_compile_testing/double_2pass_shock_propagation_stepper.cpp
  src/simulator_types_compile_testing/double_constraint_based_shock_propagation_stepper.cpp
  src/simulator_types_compile_testing/double_dynamics_projection_stepper.cpp
  src/simulator_types_compile_testing/double_dynamics_stepper.cpp
  src/simulator_types_compile_testing/double_first_order_stepper.cpp
  src/simulator_types_compile_testing/double_iterate_once_collision_resolver.cpp
  src/simulator_types_compile_testing/double_sequential_collision_resolver.cpp
  src/simulator_types_compile_testing/double_sequential_truncating_collision_resolver.cpp
  src/simulator_types_compile_testing/float_2pass_shock_propagation_stepper.cpp
  src/simulator_types_compile_testing/float_constraint_based_shock_propagation_stepper.cpp
  src/simulator_types_compile_testing/float_dynamics_projection_stepper.cpp
  src/simulator_types_compile_testing/float_dynamics_stepper.cpp
  src/simulator_types_compile_testing/float_first_order_stepper.cpp
  src/simulator_types_compile_testing/float_iterate_once_collision_resolver.cpp
  src/simulator_types_compile_testing/float_sequential_collision_resolver.cpp
  src/simulator_types_compile_testing/float_sequential_truncating_collision_resolver.cpp
)

add_library(unit_multibody_build_test ${SRC_FILES})
target_link_libraries(unit_multibody_build_test
  PRIVATE
    Boost::unit_test_framework
    Qhull::libqhull
    OpenTissue
)

#----------------------------------------------------------------------
#
# This is the only executable in the retro unit test
#
add_executable(unit_multibody
  src/unit_retro.cpp
  src/projected_gauss_seidel_compile_test.cpp
  src/math_policies_compile_test.cpp
  src/matrix_setup.h
  src/compile_test.cpp
)

target_link_libraries(unit_multibody
  PRIVATE
    Boost::unit_test_framework
    Qhull::libqhull
    OpenTissue
)

install(
  TARGETS unit_multibody
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_multibody)
