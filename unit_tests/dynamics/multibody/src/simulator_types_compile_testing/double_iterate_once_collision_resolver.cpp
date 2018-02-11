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

typedef OpenTissue::mbd::default_ublas_math_policy<double>  math_types;

void (*case6_ptr1)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::BisectionStepSimulator> );
void (*case6_ptr2)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*case6_ptr3)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*case6_ptr4)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::FixPointStepSimulator> );
void (*case6_ptr5)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*case6_ptr6)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*case6_ptr7)() = &(simulator_type_compile_test<math_types, test6,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );

