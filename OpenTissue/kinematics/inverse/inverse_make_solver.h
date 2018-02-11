#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_MAKE_SOLVER_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_MAKE_SOLVER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/inverse/inverse_nonlinear_solver.h>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Make Inverse Kinematics Nonlinear Solver.
      * This convenience function assist one in creating and initializing a
      * non-linear inverse kinemtics solver. The function will create an inverse
      * kinmatics solver for a specified given skeleton by writting
      * \code
      * solver_type solver = OpenTissue::kinematics::inverse::make_solver( skeleton );
      * \endcode
      * This convenience function will tie the skeleton to the solver and by
      * default create inverse kinematic chains. The skeleton three structure
      * is used as the basis for the chain creation. Every leaf bone will
      * correspond to one end-effector in one chain and all chains will share
      * the same root bone. Afterwards one can iterate through the created inverse kinematic chains.
      * \code
      * solver_type::chain_iterator chain = solver.chain_begin();
      * solver_type::chain_iterator end   = solver.chain_end();
      * for(;chain!=end;++chain)
      * {
      *    ... do something with chain ...
      * }
      * \endcode
      * One can store the chain iterators for later manipulation. During a
      * simulation loop one would typical use the chain iterators to set up
      * desired goal placments for the end-effector of a chain.
      *
      * @param skeleton    The skeleton that should be used to initialize the new solver.
      *
      * @return            The new solver.
      */
      template<typename skeleton_type>
      inline NonlinearSolver<skeleton_type>  make_solver( skeleton_type const & skeleton )
      {
        typedef NonlinearSolver<skeleton_type>                solver_type;
        typedef typename solver_type::chain_type              chain_type;
        typedef typename skeleton_type::const_bone_iterator   const_bone_iterator;
        typedef typename skeleton_type::bone_type             bone_type;

        solver_type solver;

        solver.init( skeleton );
        const_bone_iterator    bone = skeleton.begin();  
        const_bone_iterator    end  = skeleton.end();
        for( ; bone!=end; ++bone)
        {
          if(!bone->is_leaf())
            continue;

          chain_type chain;
          chain.init( skeleton.root(), &(*bone) );

          solver.add_chain( chain );
        }
        return solver;
      }

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_MAKE_SOLVER_H
#endif
