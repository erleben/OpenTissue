#----------------------------------------------------------------------
#
# We got a bunch of compile tests, it really make no sense to build an
# executable from these. We just create a static lib for each of these.
#
set(SRC_FILES double_2pass_shock_propagation_stepper.cpp
  double_constraint_based_shock_propagation_stepper.cpp
  double_dynamics_projection_stepper.cpp
  double_dynamics_stepper.cpp
  double_first_order_stepper.cpp
  double_iterate_once_collision_resolver.cpp
  double_sequential_collision_resolver.cpp
  double_sequential_truncating_collision_resolver.cpp
  float_2pass_shock_propagation_stepper.cpp
  float_constraint_based_shock_propagation_stepper.cpp
  float_dynamics_projection_stepper.cpp
  float_dynamics_stepper.cpp
  float_first_order_stepper.cpp
  float_iterate_once_collision_resolver.cpp
  float_sequential_collision_resolver.cpp
  float_sequential_truncating_collision_resolver.cpp
)

foreach(file ${FILES})
  get_filename_component(FILE_NAME ${file} NAME_WE)
  add_library(unit_mbd_${FILE_NAME} ${file})
  target_link_libraries(unit_mbd_${FILE_NAME}
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)
endforeach()

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
    OpenTissue
)

install(
  TARGETS unit_multibody
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_multibody)
