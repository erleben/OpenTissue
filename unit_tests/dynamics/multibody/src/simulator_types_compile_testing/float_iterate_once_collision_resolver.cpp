//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include "simulator_type_compile_test.h"

template< typename types  >
class test6
  : public OpenTissue::mbd::IterateOnceCollisionResolver< types, OpenTissue::mbd::collision_laws::FrictionalNewtonCollisionLawPolicy >
{};

typedef OpenTissue::mbd::default_ublas_math_policy<float> math_types;

void (*fcase6_ptr1)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::BisectionStepSimulator> );
void (*fcase6_ptr2)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*fcase6_ptr3)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*fcase6_ptr4)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::FixPointStepSimulator> );
void (*fcase6_ptr5)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*fcase6_ptr6)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*fcase6_ptr7)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );

