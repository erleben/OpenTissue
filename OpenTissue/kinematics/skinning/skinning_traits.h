#ifndef OPENTISSUE_KINEMATICS_SKINNING_SKINNING_SKIN_TRAITS_H
#define OPENTISSUE_KINEMATICS_SKINNING_SKINNING_SKIN_TRAITS_H
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
  namespace skinning
  {

    template<typename types>
    class SkinVertexTraits
    {
    public:

      typedef typename types::vector3_type		vector3_type;
      typedef typename types::real_type			  real_type;

    public:
      vector3_type      m_coord;
      vector3_type      m_normal;
      real_type         m_u;
      real_type         m_v;
      vector3_type      m_color;
      int               m_tag;

      //--- Members used for skinning
      vector3_type      m_original_coord;	// Moved by spreak - now we can use vertex arrrays :-)
      vector3_type      m_original_normal;

      size_t			      m_key;			// Used by SBS skinning
      int               m_influences; ///< The number of bone influences
      size_t			      m_bone[4];    ///< The bone number of the influences
      real_type         m_weight[4];  ///< The weight of the influences.
      // Note: weights/bones reset to 4 from 6 by spreak (to enable GPU skinning)
    };

    template<typename types>
    class SkinFaceTraits
    {
    public:

    };

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_SKINNING_SKIN_TRAITS_H
#endif
