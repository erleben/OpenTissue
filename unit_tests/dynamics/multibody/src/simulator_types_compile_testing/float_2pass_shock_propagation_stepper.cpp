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

typedef OpenTissue::mbd::default_ublas_math_policy<float> math_types;

void (*fcase5_ptr1)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::BisectionStepSimulator> );
void (*fcase5_ptr2)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*fcase5_ptr3)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*fcase5_ptr4)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::FixPointStepSimulator> );
void (*fcase5_ptr5)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*fcase5_ptr6)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
//void (*fcase5_ptr7)() = &(simulator_type_compile_test<math_types, test5,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );

