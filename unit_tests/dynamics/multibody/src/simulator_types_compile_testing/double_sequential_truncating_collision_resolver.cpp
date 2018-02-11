//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include "simulator_type_compile_test.h"

template< typename types  >
class test8
  : public OpenTissue::mbd::SequentialTruncatingCollisionResolver< types, OpenTissue::mbd::collision_laws::FrictionalNewtonCollisionLawPolicy >
{};

typedef OpenTissue::mbd::default_ublas_math_policy<double>  math_types;

void (*case8_ptr1)() = &(simulator_type_compile_test<math_types, test8,OpenTissue::mbd::BisectionStepSimulator> );
void (*case8_ptr2)() = &(simulator_type_compile_test<math_types, test8,OpenTissue::mbd::ExplicitFixedStepSimulator> );
void (*case8_ptr3)() = &(simulator_type_compile_test<math_types, test8,OpenTissue::mbd::ExplicitSeparateErrorCorrectionFixedStepSimulator> );
void (*case8_ptr4)() = &(simulator_type_compile_test<math_types, test8,OpenTissue::mbd::FixPointStepSimulator> );
void (*case8_ptr5)() = &(simulator_type_compile_test<math_types, test8,OpenTissue::mbd::ImplicitFixedStepSimulator> );
void (*case8_ptr6)() = &(simulator_type_compile_test<math_types, test8,OpenTissue::mbd::SemiImplicitFixedStepSimulator> );
void (*case8_ptr7)() = &(simulator_type_compile_test<math_types, test8,OpenTissue::mbd::SeparatedCollisionContactFixedStepSimulator> );
