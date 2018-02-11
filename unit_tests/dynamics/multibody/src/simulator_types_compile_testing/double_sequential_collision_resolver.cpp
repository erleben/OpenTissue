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

typedef OpenTissue::mbd::default_ublas_math_policy<double>  math_types;

void (*case7_ptr1)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::BisectionStepSimulator> );
void (*case7_ptr2)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*case7_ptr3)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*case7_ptr4)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::FixPointStepSimulator> );
void (*case7_ptr5)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*case7_ptr6)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*case7_ptr7)() = &(simulator_type_compile_test<math_types, test7,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );

