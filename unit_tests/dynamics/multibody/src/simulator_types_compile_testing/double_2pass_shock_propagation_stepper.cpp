//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include "simulator_type_compile_test.h"

template< typename types  >
class test5
  : public OpenTissue::mbd::TwoPassShockPropagationStepper< types, OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy> >
{};


typedef OpenTissue::mbd::default_ublas_math_policy<double>  math_types;

void (*case5_ptr1)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::BisectionStepSimulator> );
void (*case5_ptr2)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*case5_ptr3)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*case5_ptr4)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::FixPointStepSimulator> );
void (*case5_ptr5)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*case5_ptr6)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
//void (*case5_ptr7)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );

