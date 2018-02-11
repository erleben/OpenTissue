//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include "simulator_type_compile_test.h"

template< typename types  >
class test3
  : public OpenTissue::mbd::DynamicsProjectionStepper< types, OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy> >
{};

typedef OpenTissue::mbd::default_ublas_math_policy<double>  math_types;

void (*case3_ptr1)() = &(simulator_type_compile_test<math_types, test3,OpenTissue::mbd::BisectionStepSimulator> );
void (*case3_ptr2)() = &(simulator_type_compile_test<math_types, test3,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*case3_ptr3)() = &(simulator_type_compile_test<math_types, test3,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*case3_ptr4)() = &(simulator_type_compile_test<math_types, test3,OpenTissue::mbd::FixPointStepSimulator> );
void (*case3_ptr5)() = &(simulator_type_compile_test<math_types, test3,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*case3_ptr6)() = &(simulator_type_compile_test<math_types, test3,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*case3_ptr7)() = &(simulator_type_compile_test<math_types, test3,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );
