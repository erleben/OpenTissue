//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include "simulator_type_compile_test.h"

template< typename types  >
class test2
  : public OpenTissue::mbd::FirstOrderStepper< types, OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy> >
{};

typedef OpenTissue::mbd::default_ublas_math_policy<double>  math_types;

void (*case2_ptr1)() = &(simulator_type_compile_test<math_types, test2,OpenTissue::mbd::BisectionStepSimulator> );
void (*case2_ptr2)() = &(simulator_type_compile_test<math_types, test2,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*case2_ptr3)() = &(simulator_type_compile_test<math_types, test2,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*case2_ptr4)() = &(simulator_type_compile_test<math_types, test2,OpenTissue::mbd::FixPointStepSimulator> );
void (*case2_ptr5)() = &(simulator_type_compile_test<math_types, test2,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*case2_ptr6)() = &(simulator_type_compile_test<math_types, test2,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*case2_ptr7)() = &(simulator_type_compile_test<math_types, test2,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );
