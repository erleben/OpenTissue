#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_JACOBIAN_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_JACOBIAN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <OpenTissue/kinematics/inverse/inverse_accessor.h>

#include <vector>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Compute Jacobian For the Kinematic Chains.
      * This function will first determine the size of the Jacobian
      * matrix and the non-zero block pattern layout of the Jacobian
      * matrix. Following this the function will fill in values into
      * the Jacobian matrix.
      *
      * It is implicitly assumed that all the kinematics chains are
      * comming from the same skeleton.
      *
      * @param chain_begin    An iterator to the first chain.
      * @param chain_end      An iterator to one past the last chain.
      * @param bone_begin     An iterator to the first bone.
      * @param bone_end       An iterator to one past the last bone.
      * @param J              Upon return this argument will hold the Jacobian
      *                       corresponding to all the chains in the specified
      *                       sequence.
      */
      template <  typename chain_iterator, typename bone_iterator, typename matrix_type>
      inline void compute_jacobian(
          chain_iterator const & chain_begin
        , chain_iterator const & chain_end
        , bone_iterator const & bone_begin
        , bone_iterator const & bone_end
        , matrix_type & J
        )
      {
        typedef typename matrix_type::value_type                     real_type;
        typedef typename ublas::vector<real_type>                    vector_type;
        typedef typename ublas::matrix_range< matrix_type >          matrix_range;

        typedef typename chain_iterator::value_type                  chain_type;
        typedef typename chain_type::bone_traits                     bone_traits;

        // Determine how many chains we have
        size_t max_chains = std::distance( chain_begin, chain_end );
        size_t max_bones  = std::distance( bone_begin, bone_end   );

        // Determine how many rows and columns we have in the Jacobian
        std::vector<size_t> row_index(max_chains);      ///< Entry i holds the starting row index of the i'th chain.
        std::vector<size_t> column_index(max_bones);    ///< The i'th entry holds the starting column index of the i'th bone

        size_t rows    = 0;              ///< The number of rows in the Jacobian
        size_t columns = 0;              ///< The number of columns in the Jacobian

        std::vector<size_t> dof_array(max_bones);   ///< The i'th entry holds the number of degrees of freedom of the i'th bone
        {
          size_t index = 0;
          for(chain_iterator chain = chain_begin;chain!=chain_end;++chain)
          {
            row_index[index++] = rows;
            rows += chain->get_goal_dimension();
          }

          bone_iterator bone     = bone_begin;
          for(;bone!=bone_end;++bone)
          {
            dof_array[ bone->get_number()] = bone->active_dofs();
          }

          for (size_t i=0;i<max_bones;++i)
          {
            column_index[i] = columns;
            columns += dof_array[i];
          }
        }

        // Allocate space for Jacobian
        if(J.size1() != rows || J.size2() != columns)
        {
          J.resize( rows, columns, false);
        }
        J.clear(); // Empty all data from J 

        // Now we are ready to fill in data into the Jacobian matrix.
        size_t chain_index = 0;
        for(chain_iterator chain = chain_begin;chain!=chain_end;++chain)
        {
          typename chain_type::bone_iterator bend  = chain->bone_end();
          typename chain_type::bone_iterator bone  = chain->bone_begin();

          for(; bone != bend ; ++bone)
          {
            size_t bone_index     = bone->get_number();
            size_t begin_row      = row_index[chain_index];
            size_t end_row        = begin_row + chain->get_goal_dimension();
            size_t begin_column   = column_index[bone_index];
            size_t end_column     = begin_column + dof_array[bone_index];

            matrix_range J_block = ublas::subrange(
              J
              , begin_row
              , end_row
              , begin_column
              , end_column
              );

            ACCESSOR::compute_jacobian( (*bone) , (*chain), J_block); 
          }
          ++chain_index;
        }

      }

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_COMPUTE_JACOBIAN_H
#endif
