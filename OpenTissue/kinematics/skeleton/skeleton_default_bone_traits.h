#ifndef OPENTISSUE_KINEMATICS_SKELETON_DEFAULT_BONE_TRAITS_H
#define OPENTISSUE_KINEMATICS_SKELETON_DEFAULT_BONE_TRAITS_H
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
  namespace skeleton
  {

    /**
    * Default Bone Traits.
    * A bone transformation can be represented in many different ways. For instance
    * using homogeneous matrices. In our default implementation we have chosen a compound
    * type consisting of a vector and a quaternion. This type is called a coordsys_type.
    *
    * The bone traits class offers functionality to work with different representations
    * of the bone transformations.
    *
    */
    template<typename math_types_>
    class DefaultBoneTraits 
    {
    public:

      typedef math_types_                             math_types;
      typedef typename math_types::coordsys_type      transform_type;
      typedef typename math_types::coordsys_type      coordsys_type;
      typedef typename math_types::vector3_type       vector3_type;

      /**
       * Convert.
       * The convert function should be able to convert back and forth
       * between a transform_type and a coordsys_type. In our case where
       * our transform_type is simply a coordsys_type we can settle with
       * a single and very simple implementation of the ``convert'' method.
       * In a general setting one would probably have to make two overloaded
       * versions of the convert function.
       *
       * @param T    A value represented in the input type.
       *
       * @return     The value in the output type.
       */
      static coordsys_type convert(coordsys_type const & T)
      {
        return T;
      }

      /**
       * Retrive Identity Transformation.
       *
       * @return   Upon return the identity transformation is returned.
       */
      static transform_type get_identity_transform()
      {
        static transform_type T;// default constructed coordsys is set to identity transformation.
        return T;
      }

      /**
       * Compute absolute pose transformation.
       *
       * @param parent     The absolute transformation of the parent bone.
       * @param relative   The relative transformation of the bone.
       *
       * @return           The absolute transformation of the bone.
       */
      static transform_type compute_absolute_pose_transform( transform_type const & parent, transform_type const & relative )
      {
        transform_type absolute;
        absolute.T() = parent.Q().rotate( relative.T() ) + parent.T();
        absolute.Q() = prod( parent.Q() , relative.Q() );
        return absolute;
      }

      /**
       * Compute Bone Space Transformation.
       *
       * @param absolute     The absolute transformation of the bone.
       * @param bone_space   The bone space location of the bone.
       *
       * @return             The bone space transformation.
       */
      static transform_type compute_bone_space_transform( transform_type const & absolute, transform_type const & bone_space)
      {
        transform_type bone_space_transform;
        bone_space_transform.T() = absolute.Q().rotate( bone_space.T() ) + absolute.T();
        bone_space_transform.Q() = prod( absolute.Q() , bone_space.Q() );
        return bone_space_transform;
      }

      /**
       * Transform a point by a specified transformation.
       *
       * @param T     The coordinate transformation that should be used.
       * @param x     The point that should be transformed
       *
       * @return      The transformed point.
       */
      static vector3_type transform_point(transform_type const & T, vector3_type const & x)
      {
        vector3_type tmp(x);
        T.xform_point(tmp);
        return tmp;
      }

      /**
       * Transform a vector by a specified transformation.
       *
       * @param T     The coordinate transformation that should be used.
       * @param v     The vector that should be transformed
       *
       * @return      The transformed vector.
       */
      static vector3_type transform_vector(transform_type const & T, vector3_type const & v)
      {
        vector3_type tmp(v);
        T.xform_vector(tmp);
        return tmp;
      }

    };

  } // namespace skeleton
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKELETON_DEFAULT_BONE_TRAITS_H
#endif
