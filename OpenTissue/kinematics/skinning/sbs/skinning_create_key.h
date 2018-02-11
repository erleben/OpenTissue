#ifndef OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_CREATE_KEY_H
#define OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_CREATE_KEY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

// Center of rotation by Kasper A. Andersen @ DIKU, spreak@spreak.dk

namespace OpenTissue
{
  namespace skinning
  {

    template<typename vertex_type>
    static size_t create_key( const vertex_type & vertex )
    {
      typedef std::vector<size_t>	key_sort_type;

      key_sort_type keys;	

      for(int i= 0; i < vertex->m_influences; ++i)
        keys.push_back( vertex->m_bone[i] );

      std::sort( keys.begin(), keys.end() );

      size_t key = 0;
      for(int i=0; i < vertex->m_influences; ++i)
        key = (key + (keys[i]+1)) * 100;

      return key;
    }

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_CREATE_KEY_H
#endif
