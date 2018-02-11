#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


/**
@file inverse.h
@brief Inverse kinematics grand include header.

@author Kenny Erleben
@date July 2008

@warning  To reduce compile times it is better not to use the include-all
header file. Instead include only the exact header files that
one needs.

This include header file is intended for the user that
do not wont to know the particularities of the inverse
kinematics library. The header file basically makes sure
to include all other header files relevant to the inverse
kinematics library.

To use the Inverse kinematics library one would write
\code
#include<OpenTissue/kinematics/inverse/inverse.h>
\endcode
Inverse kinematics is applied to a skeleton, so one needs a
skeleton data structure to get started. The skeleton data
library can be included by writting
\code
#include <OpenTissue/kinematics/skeleton/skeleton_types.h>
\endcode
Now one is ready for creating the first types needed by the
inverse kinematics library.
\code
typedef ... math_types
typedef OpenTissue::skeleton::DefaultBoneTraits<math_types>              bone_traits;
\endcode
These two typedefs simply defines what kind of precision, vectors,
quaternion and matrix types one wants to use. The bone_tratis type
is needed next to form a basis for the specialized inverse kinematics
bone traits.
\code
typedef OpenTissue::kinematics::inverse::BoneTraits< bone_traits >       ik_bone_traits;
\endcode
Now we have a bone traits type that is compatible with the inverse
kinematics library. Next one can create a skeleton type
\code
typedef OpenTissue::skeleton::Types<math_types,ik_bone_traits>           skeleton_types;
typedef skeleton_types::skeleton_type                                    skeleton_type;
\endcode
Finally all that is needed is to create an inverse kinematics solver
type. This is done by telling what skeleton type we want to work with.
\code
typedef OpenTissue::kinematics::inverse::NonlinearSolver<skeleton_type>  solver_type;
\endcode
Now we are ready to use the types. First we will construct a skeleton
\code
skeleton_type skeleton;
// ... initialize skeleton by adding bones or importing data form some file ...
\endcode
Next we need to tell each bone of the skeleton what type it should be
\code
skeleton_type::bone_type * bone = ... retrieve bone pointer from skeleton ...;
bone->type() = ik_bone_traits::ball_type;
... repeat for all other bones in skeleton ...
\endcode
Hereafter one can invoke a convenience function which will assist
one in completing the initialization of the skeleton and the bones.
\code
OpenTissue::kinematics::inverse::set_joint_parameters( skeleton );
\endcode
The method will try to determine joint axes and initial
joint parameter values. Now we got a skeleton instance that
is initialized and ready for usage. Next we will create an
inverse kinmatics solver for the skeleton. Once more we will
use a convenience function to assist us in this task.
\code
solver_type solver = OpenTissue::kinematics::inverse::make_solver( skeleton );
\endcode
This convenience function will tie the skeleton to the solver and by
default create inverse kinematic chains. The skeleton three structure
is used as the basis for the chain creation. Every leaf bone will
correspond to one end-effector in one chain and all chains will share
the same root bone. Afterwards one can iterate through the created inverse kinematic chains.
\code
solver_type::chain_iterator chain = solver.chain_begin();
solver_type::chain_iterator end   = solver.chain_end();
for(;chain!=end;++chain)
{
   ... do something with chain ...
}
\endcode
One can store the chain iterators for later manipulation. During a
simulation loop one would typical use the chain iterators to set up
desired goal placments for the end-effector of a chain. Here is a code
snippet illustrating how to do this.
\code
chain_iterator chain = ....;

chain->p_global() = vector3_type(....);
chain->x_global() = vector3_type(....);
chain->y_global() = vector3_type(....);
\endcode
After having specified the goal placements one is ready for solving the
inverse kinmatics problem.
\code
solver.solve()
\endcode
That is it. Next one can render the skin og skeleton and/or the bones.
Notice that upon return from the solve method the skeleton will have
been updated such that both the relative and absolute bone transforms
match the solution found for the inverse kinematics problem. Meaning that
there is no need to invoke a compute_pose method on the skeleton prior
to performing any rendering.

*/

// Core library data structures and functionality
#include <OpenTissue/kinematics/inverse/inverse_accessor.h>
#include <OpenTissue/kinematics/inverse/inverse_bone_traits.h>
#include <OpenTissue/kinematics/inverse/inverse_chain.h>
#include <OpenTissue/kinematics/inverse/inverse_compute_jacobian.h>
#include <OpenTissue/kinematics/inverse/inverse_nonlinear_solver.h>

// Convenience and utility functions for making life a little easier
#include <OpenTissue/kinematics/inverse/inverse_compute_weighted_difference.h>
#include <OpenTissue/kinematics/inverse/inverse_set_joint_parameters.h>
#include <OpenTissue/kinematics/inverse/inverse_get_joint_parameters.h>
#include <OpenTissue/kinematics/inverse/inverse_make_solver.h>
#include <OpenTissue/kinematics/inverse/inverse_add_chain.h>
#include <OpenTissue/kinematics/inverse/inverse_compute_joint_limits_projection.h>
#include <OpenTissue/kinematics/inverse/inverse_write_benchmarks.h>

#include <OpenTissue/kinematics/inverse/io/inverse_xml_read.h>
#include <OpenTissue/kinematics/inverse/io/inverse_xml_write.h>


//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_H
#endif
