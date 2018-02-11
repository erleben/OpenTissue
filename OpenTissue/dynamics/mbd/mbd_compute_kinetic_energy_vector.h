#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_KINETIC_ENERGY_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_KINETIC_ENERGY_VECTOR_H
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
  namespace mbd
  {
    /**
    * Compute Kinetic Energy Vector.
    * Retrieves the total kinetic energy of a sequence of bodies and stores these into a vector.
    *
    * This template function is usefull for debugging. For instance to verify that kinetic
    * energy is not increased during a simulation.
    *
    * Example usage:
    *
    *  vector_type energy;
    *  mbd::compute_kinetic_energy_vector(configuration.body_begin(),configuration.body_end(),energy);
    *  std::cout << "E = " << energy << ";" << std:endl;
    *
    */
    template< typename indirect_body_iterator,typename vector_type>
     void compute_kinetic_energy_vector(indirect_body_iterator begin, indirect_body_iterator end, vector_type & energy)
    {
      typedef typename indirect_body_iterator::value_type     body_type;
      typedef typename body_type::math_policy                 math_policy;
      typedef typename vector_type::size_type                 size_type;

      size_type n = std::distance(begin,end);

      math_policy::resize( energy, n);

      size_type i = 0;
      for(indirect_body_iterator body=begin;body!=end;++body,++i)
        energy(i) = body->compute_kinetic_energy();
    }

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_KINETIC_ENERGY_VECTOR_H
#endif
