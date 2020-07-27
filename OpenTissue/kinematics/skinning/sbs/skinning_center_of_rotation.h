#ifndef OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_CENTER_OF_ROTATION_H
#define OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_CENTER_OF_ROTATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h> 
#include <OpenTissue/core/math/big/big_svd.h> 
#include <OpenTissue/core/math/math_quaternion.h>
#include <OpenTissue/core/math/math_vector3.h>

// Center of rotation by Kasper A. Andersen @ DIKU, spreak@spreak.dk

namespace OpenTissue
{
  namespace skinning
  {

    template<typename value_type>
    value_type
      bsum( value_type val )
    {
      value_type result = 0;
      for( size_t i=1; i<val; ++i )
        result += i;
      return result;
    }

    /**
    * Center of rotation computation for a collections of bones in a skeleton.
    *
    * @param skeleton  The skeleton containing bones = coordinate systems.
    * @param bones		Container storing (pointers to) the bones to compute rotation
    *					rotation centers for.
    */
    template<typename skeleton_type, typename container_type> 
    typename skeleton_type::math_types::vector3_type
      center_of_rotation( skeleton_type const & skeleton, container_type const & bones )
    {
      //--- Skeleton typedefs
      typedef typename skeleton_type::math_types::vector3_type		vector3_type;
      typedef typename skeleton_type::math_types::coordsys_type		coordsys_type;
      typedef typename skeleton_type::math_types::matrix3x3_type	matrix3x3_type;

      //--- SVD matrix typedefs
      typedef ublas::matrix<double>                       svd_matrix_type;
      typedef ublas::vector<double>				                svd_vector_type;

      //--- Create SVD matrix and vectors
      svd_matrix_type A;
      svd_vector_type x;
      svd_vector_type b;

      //--- Get first matrix dimension
      size_t dim_x = 3 * bsum( bones.size() ); // Fast binomial coefficient for (n,2)

      //--- Get second matrix dimension (is always 3)
      size_t dim_y = 3;

      //--- Resize matrices and vectors
      A.resize( dim_x, dim_y, false );
      x.resize( dim_y );
      b.resize( dim_x );

      //--- Get iterators
      typename container_type::const_iterator it = bones.begin();
      typename container_type::const_iterator curr_pos = bones.begin();
      typename container_type::const_iterator end = bones.end();

      //--- position to start at...
      size_t pos = 1;
      unsigned int idx = 0;

      int kc = 0;

      //--- Create 3(n,2) x 3 matrix 'lhs' and 3(n,2) vector 'rhs'  
      for(; it!=end; ++it )
      {
        // Reset position iterator
        curr_pos = bones.begin();
        for( size_t i=0; i<pos; ++i )
          ++curr_pos;

        coordsys_type const & xformA = (*it)->bone_space_transform();
        const vector3_type & transA = xformA.T(); 
        matrix3x3_type rotA( xformA.Q() ); // Convert quat to matrix

        for( ; curr_pos!=bones.end(); ++curr_pos )
        {
          coordsys_type const & xformB = (*curr_pos)->bone_space_transform();
          const vector3_type & transB = xformB.T(); 
          matrix3x3_type rotB( xformB.Q() );

          vector3_type rhs = transB - transA;
          matrix3x3_type lhs = rotA - rotB;

          //std::cout << "Lhs: " << lhs << std::endl;
          //std::cout << "Rhs: " << rhs << std::endl;

          //--- Create rhs svd vector
          b(idx) = rhs(0);
          b(idx+1) = rhs(1);
          b(idx+2) = rhs(2);

          //--- Create lhs svd matrix
          A(idx,0) = lhs(0,0); 
          A(idx,1) = lhs(0,1); 
          A(idx,2) = lhs(0,2);
          A(idx+1,0) = lhs(1,0);
          A(idx+1,1) = lhs(1,1);
          A(idx+1,2) = lhs(1,2);
          A(idx+2,0) = lhs(2,0);
          A(idx+2,1) = lhs(2,1);
          A(idx+2,2) = lhs(2,2);

          idx += 3;
          ++kc;

        }

        //--- Step one position...
        ++pos;
      }

      //--- Let LAPACK do the hard work :-)
      math::big::svd(A,x,b);

      //std::cout << "R_c: " << vector3_type( x(0), x(1), x(2) ) << std::endl;

      //if( x(0)>1e+5 )
      //	return vector3_type( 0, 0, 0 );

      return vector3_type( x(0), x(1), x(2) );
    }

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_CENTER_OF_ROTATION_H
#endif
