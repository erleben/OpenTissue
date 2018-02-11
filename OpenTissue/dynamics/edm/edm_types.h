#ifndef OPENTISSUE_DYNAMICS_EDM_EDM_TYPES_H
#define OPENTISSUE_DYNAMICS_EDM_EDM_TYPES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/dynamics/edm/edm_force.h>
#include <OpenTissue/dynamics/edm/edm_object.h>
#include <OpenTissue/dynamics/edm/edm_system.h>
#include <OpenTissue/dynamics/edm/edm_model.h>

#include <OpenTissue/utility/utility_empty_traits.h>

namespace OpenTissue
{

  namespace edm
  {

    /**
     * EDM Type Binder Class.
     * Use this class to define the sph user types.
     */
    template <
        typename math_types_
      , typename model_traits_ = OpenTissue::utility::EmptyTraits
      , typename object_traits_ = OpenTissue::utility::EmptyTraits
    >
    class Types
    {
    public:

      typedef math_types_                        math_types;
      typedef typename math_types::value_traits  value_traits;
      typedef typename math_types::real_type     real_type;
      typedef typename math_types::vector3_type  vector3_type;
      typedef model_traits_                      model_traits;
      typedef object_traits_                     object_traits;

      struct tensor1_type 
      {
        tensor1_type(){t0[0]=value_traits::zero();}
        real_type t0[1];
      };

      struct tensor2_type 
      {
        tensor2_type(){t0[0]=t0[1]=t1[0]=t1[1]=value_traits::zero();}
        real_type t0[2];
        real_type t1[2];
      };

      struct tensor3_type 
      {
        tensor3_type(){t0[0]=t0[1]=t0[2]=t1[0]=t1[1]=t1[2]=t2[0]=t2[1]=t2[2]=value_traits::zero();}
        real_type t0[3];
        real_type t1[3];
        real_type t2[3];
      };

      struct texUV 
      {
        real_type u,v;
      };


      /**
      * A generic particle as used in EDM
      */
      struct Particle
      {
        Particle()
          : m(value_traits::one())
          , g(value_traits::zero())
          , f(false)
        {}
        virtual ~Particle() {}

        vector3_type  r;  ///< current position
        vector3_type  o;  ///< old position (from last timestep)
        vector3_type  F;  ///< current external force acting on particle
        vector3_type  E;  ///< current elasticity force acting on particle
        vector3_type  n;  ///< normal
        vector3_type  v;  ///< current velocity
        texUV         t;  ///< texture coordinates (u,v)
        real_type     m;  ///< mass density (mu)
        real_type     g;  ///< damping desity (gamma)
        bool          f;  ///< is this particle constrained to it's current position
      };

      /**
      * New specified EDM model types
      */
      typedef enum {EDM_Surface, EDM_Solid} model_id_type;

      typedef Force<Types>   force_type;
      typedef Object<Types>  object_type;
      typedef System<Types>  system_type;
      typedef Model<Types>   model_type;

    };

  }  // namespace edm

}  // namespace OpenTissue


// OPENTISSUE_DYNAMICS_EDM_EDM_TYPES_H
#endif
