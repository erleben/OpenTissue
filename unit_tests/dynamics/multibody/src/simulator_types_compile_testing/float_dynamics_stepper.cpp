//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include "simulator_type_compile_test.h"

template< typename types  >
class test1
  : public OpenTissue::mbd::DynamicsStepper< types, OpenTissue::mbd::ProjectedGaussSeidel<typename types::math_policy> >
{};

typedef OpenTissue::mbd::default_ublas_math_policy<float> math_types;

void (*fcase1_ptr1)() = &(simulator_type_compile_test<math_types, test1,OpenTissue::mbd::BisectionStepSimulator> );
void (*fcase1_ptr2)() = &(simulator_type_compile_test<math_types, test1,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*fcase1_ptr3)() = &(simulator_type_compile_test<math_types, test1,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*fcase1_ptr4)() = &(simulator_type_compile_test<math_types, test1,OpenTissue::mbd::FixPointStepSimulator> );
void (*fcase1_ptr5)() = &(simulator_type_compile_test<math_types, test1,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*fcase1_ptr6)() = &(simulator_type_compile_test<math_types, test1,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*fcase1_ptr7)() = &(simulator_type_compile_test<math_types, test1,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );
