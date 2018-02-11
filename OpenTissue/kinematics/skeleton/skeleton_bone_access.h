#ifndef OPENTISSUE_KINEMATICS_SKELETON_SKELETON_BONE_ACCESS_H
#define OPENTISSUE_KINEMATICS_SKELETON_SKELETON_BONE_ACCESS_H
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
     * Bone Access Class.
     * The purpose of this class is for others to obtain direct access
     * to private and protected members of the bone class.
     *
     * It is a back-door into the Bone class and should be used with care.
     */
    class BoneAccess
    {
    public:

      /**
      * Set bone name.
      *
      * @param bone    A pointer to the bone.
      * @param name    The new name of the bone.
      *
      */
      template<typename bone_type>
      static void set_name(bone_type * bone, std::string const & name)
      {
        bone->m_name = name;
      }

      /**
      * Set bone number.
      *
      * @param bone    A pointer to the bone.
      * @param number  The new number of the bone.
      */
      template<typename bone_type>
      static void set_number(bone_type * bone, size_t const & number)
      {
        bone->m_number = number;
      }

      /**
      * Set bone parent.
      *
      * @param bone    A pointer to the bone.
      * @param parent  A pointer to the parent of the bone.
      */
      template<typename bone_type>
      static void set_parent(bone_type * bone, bone_type * parent)
      {
        bone->m_parent = parent;
      }

      /**
      * Set skeleton.
      *
      * @param bone      A pointer to the bone.
      * @param skeleton  A pointer to the skeleton that the bone belongs to.
      */
      template<typename bone_type, typename skeleton_type>
      static void set_skeleton(bone_type * bone, skeleton_type * skeleton)
      {
        bone->m_skeleton = skeleton;
      }


      /**
       * Add a child to a bone.
       *
       * @param bone   A pointer to the bone.
       * @param child  The child that should be added to the bone.
       */
      template<typename bone_type>
      static void add_child(bone_type * bone, bone_type * child)
      {
        bone->m_children.push_back( child );
      }

      /**
       * Get a pointer to the first child of the bone.
       *
       * @param bone   A pointer to the bone.
       * @return       A pointer to the first child bone or null if none exist.
       */
      template<typename bone_type>
      static bone_type * get_first_child_ptr(bone_type const * bone)
      {
        if(bone->m_children.empty())
          return 0;
        return bone->m_children.front();
      }


    };

  } // namespace skeleton
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKELETON_SKELETON_BONE_ACCESS_H
#endif
