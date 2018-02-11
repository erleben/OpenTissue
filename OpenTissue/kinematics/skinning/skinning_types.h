#ifndef OPENTISSUE_KINEMATICS_SKINNING_SKINNING_TYPES_H
#define OPENTISSUE_KINEMATICS_SKINNING_SKINNING_TYPES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/skinning/skinning_animated_skin.h>
#include <OpenTissue/kinematics/skinning/lbs/skinning_lbs.h>
#include <OpenTissue/kinematics/skinning/lbs/skinning_lbs_gpu.h>
#include <OpenTissue/kinematics/skinning/sbs/skinning_sbs.h>
#include <OpenTissue/kinematics/skinning/sbs/skinning_sbs_gpu.h>
#include <OpenTissue/kinematics/skinning/dbs/skinning_dbs.h>
#include <OpenTissue/kinematics/skinning/gl_skin_render.h>
#include <OpenTissue/utility/utility_material.h>

namespace OpenTissue
{
  namespace skinning
  {


    template <typename math_types_, template <typename> class skin_part_type_>
    class Types
    {
    public:

      typedef Types<math_types_,skin_part_type_>  skin_types;
      typedef math_types_                         math_types;
      typedef skin_part_type_<math_types>         skin_part_type;
      typedef AnimatedSkin<skin_types>            skin_type;
      typedef typename gl::SkinRender             skin_render_type;
      typedef OpenTissue::utility::Material       material_type;
      typedef typename math_types::index_type     key_type;
    };

  } // namespace skinning
} // namespace OpenTissue


//OPENTISSUE_KINEMATICS_SKINNING_SKINNING_TYPES_H
#endif
