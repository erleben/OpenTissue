//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include "simulator_type_compile_test.h"

template< typename types  >
class test7
  : public OpenTissue::mbd::SequentialCollisionResolver< types, OpenTissue::mbd::collision_laws::FrictionalNewtonCollisionLawPolicy >
{};

typedef OpenTissue::mbd::default_ublas_math_policy<float> math_types;


void (*fcase7_ptr1)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::BisectionStepSimulator> );
void (*fcase7_ptr2)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*fcase7_ptr3)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*fcase7_ptr4)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::FixPointStepSimulator> );
void (*fcase7_ptr5)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*fcase7_ptr6)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*fcase7_ptr7)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );

