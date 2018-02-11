#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_MASS_MATRIX_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_MASS_MATRIX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Extract Generalized Mass Matrix.
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param M  Upon return this arguement contains the generalized mass matrix.
    */
    template<typename indirect_body_iterator,typename matrix_type>
    void get_mass_matrix(indirect_body_iterator begin, indirect_body_iterator end, matrix_type & M)
    {
      typedef typename indirect_body_iterator::value_type  body_type;
      typedef typename body_type::math_policy              math_policy;
      typedef typename body_type::matrix3x3_type           matrix3x3_type;
      typedef typename matrix_type::size_type              size_type;
      typedef typename matrix_type::value_type             real_type;

      matrix3x3_type I;

      size_type n = std::distance(begin,end);

      math_policy::resize( M, 6*n,6*n);      
      M.clear();
      size_type tag=0;

      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"get_mass_matrix(): body was not active");

        size_type offset = tag*6;
        body->m_tag = tag++;

        real_type mass = body->get_mass();

        body->get_inertia_wcs(I);

        assert(is_number(mass)   || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(0,0)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(0,1)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(0,2)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(1,0)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(1,1)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(1,2)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(2,0)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(2,1)) || !"get_mass_matrix(): non number encountered");
        assert(is_number(I(2,2)) || !"get_mass_matrix(): non number encountered");

        M(offset,offset)     = mass;
        M(offset+1,offset+1) = mass;
        M(offset+2,offset+2) = mass;
        offset += 3;      
        M(offset,offset)     = I(0,0);
        M(offset,offset+1)   = I(0,1);
        M(offset,offset+2)   = I(0,2);
        M(offset+1,offset)   = I(1,0);
        M(offset+1,offset+1) = I(1,1);
        M(offset+1,offset+2) = I(1,2);
        M(offset+2,offset)   = I(2,0);
        M(offset+2,offset+1) = I(2,1);
        M(offset+2,offset+2) = I(2,2);
      }
    }

    template<typename group_type,typename matrix_type>
    void get_mass_matrix(group_type const & group, matrix_type & M)
    {
      get_mass_matrix(group.body_begin(),group.body_end(),M);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_MASS_MATRIX_H
#endif
