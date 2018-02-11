#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_INVERSE_MASS_MATRIX_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_INVERSE_MASS_MATRIX_H
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
    * Extract Generalized Inverse Mass Matrix.
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param M       Upon return this arguement contains the generalized inverse mass matrix.
    */
    template<typename indirect_body_iterator,typename matrix_type>
    void get_inverse_mass_matrix(indirect_body_iterator begin, indirect_body_iterator end, matrix_type & invM)
    {
      typedef typename indirect_body_iterator::value_type  body_type;
      typedef typename body_type::matrix3x3_type           matrix3x3_type;
      typedef typename body_type::math_policy              math_policy;
      typedef typename matrix_type::size_type              size_type;
      typedef typename matrix_type::value_type             real_type;

      matrix3x3_type invI;

      size_type n = std::distance(begin,end);

      math_policy::resize(invM,6*n,6*n);
      invM.clear();
      size_type tag=0;

      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"get_inverse_mass_matrix(): body was not active");

        size_type offset = tag*6;
        body->m_tag = tag++;

        real_type inv_mass = body->get_inverse_mass();

        body->get_inverse_inertia_wcs(invI);

        assert(is_number(inv_mass)  || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(0,0)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(0,1)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(0,2)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(1,0)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(1,1)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(1,2)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(2,0)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(2,1)) || !"get_inverse_mass_matrix(): non number encountered");
        assert(is_number(invI(2,2)) || !"get_inverse_mass_matrix(): non number encountered");

        invM(offset,offset)     = inv_mass;
        invM(offset+1,offset+1) = inv_mass;
        invM(offset+2,offset+2) = inv_mass;
        offset += 3;      
        invM(offset,offset)     = invI(0,0);
        invM(offset,offset+1)   = invI(0,1);
        invM(offset,offset+2)   = invI(0,2);
        invM(offset+1,offset)   = invI(1,0);
        invM(offset+1,offset+1) = invI(1,1);
        invM(offset+1,offset+2) = invI(1,2);
        invM(offset+2,offset)   = invI(2,0);
        invM(offset+2,offset+1) = invI(2,1);
        invM(offset+2,offset+2) = invI(2,2);
      }
    }

    template<typename group_type,typename matrix_type>
    void get_inverse_mass_matrix(group_type const & group, matrix_type & invM)
    {
      get_inverse_mass_matrix(group.body_begin(),group.body_end(),invM);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_INVERSE_MASS_MATRIX_H
#endif
