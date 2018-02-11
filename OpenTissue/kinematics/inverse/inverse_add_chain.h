#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_ADD_CHAIN_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_ADD_CHAIN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Add Chain to Inverse Kinematics Solver.
      * This convenience function will create a new chain and add it to the
      * specified solver. Obsreve that a chain is a connected sequence of
      * bones. The root of the chain does not have to be the root bone
      * of the skeleton. The end-bone of the chain sequence is called the
      * end-effector of the chain.
      *
      * @param solver          The solver that the new chain should be added to.
      * @param root            A reference to the root of the new chain
      * @param end_effector    The end-effector of the new chain.
      *
      * @return                 An iterator to the newly added chain.
      */
      template<typename solver_type>
      inline typename solver_type::chain_iterator add_chain(
          solver_type & solver
        , typename solver_type::skeleton_type::bone_type const & root_
        , typename solver_type::skeleton_type::bone_type const & end_effector
        )
      {
        typedef typename solver_type::chain_type                 chain_type;
        typedef typename solver_type::skeleton_type::bone_type   bone_type;
      
        bone_type * end  = const_cast<bone_type*>(&end_effector);
        bone_type * root = const_cast<bone_type*>(&root_ );
        
        chain_type chain;
        chain.init( root, end );
      
        typename solver_type::chain_iterator iter = solver.add_chain( chain );
            
        return iter;
      }

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_ADD_CHAIN_H
#endif
