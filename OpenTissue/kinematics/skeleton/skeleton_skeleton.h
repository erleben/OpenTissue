#ifndef OPENTISSUE_KINEMATICS_SKELETON_SKELETON_H
#define OPENTISSUE_KINEMATICS_SKELETON_SKELETON_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/skeleton/skeleton_bone_access.h>

#include <list>
#include <vector>
#include <map>
#include <cassert>
#include <cmath>

namespace OpenTissue
{
  namespace skeleton
  {
    /**
    * A Skeleton Class.
    * A skeleton is a collection of bones. One bone is designated as the root
    * bone, all other bones are connected such that they form a recursive tree
    * structure with the root bone as the root.
    *
    * This class provides an interface for building up the skeleton in a
    * consistent manner. While during so bones are assigned unique numbers
    * identifying their placement in the hierarchical tree structure. It
    * is ensured that parents always have a bone number less than their
    * children or any descendants.
    *
    * The class also provides internal book-keeping which provides the means
    * for accessing bones either by their names or by their bone number.
    *
    */
    template<typename types>
    class Skeleton
      : public types::skeleton_traits
    {
    public:

      typedef typename types::math_types                              math_types;
      typedef typename types::bone_traits                             bone_traits;

    protected:

      typedef typename math_types::value_traits                       value_traits;
      typedef typename math_types::vector3_type                       vector3_type;
      typedef typename math_types::quaternion_type                    quaternion_type;
      typedef typename math_types::real_type                          real_type;

    public:

      typedef typename types::bone_type                               bone_type;
      typedef typename types::skeleton_type                           skeleton_type;

    protected:

      typedef std::list<bone_type>                                    bone_container;
      typedef std::map<std::string,size_t>                            bone_number_lut_container;
      typedef std::vector<bone_type*>                                 bone_ptr_lut_container;
      typedef typename bone_number_lut_container::iterator            lut_iterator;

    public:

      typedef typename bone_container::iterator                       bone_iterator;
      typedef typename bone_container::const_iterator                 const_bone_iterator;

    protected:

      bone_container                    m_bones;                ///< All bones in skeleton
      bone_number_lut_container         m_bone_number_lut;      ///< Lookup table to get bone number from bone name.
      bone_ptr_lut_container            m_bone_ptr_lut;         ///< Lookup table to get bone pointer from bone number.
      bone_type *                       m_root;                 ///< Pointer to root bone.

    public:

      Skeleton()
        : m_root(0) 
      {}

    public:

      /**
      * @return   An iterator to the position of the first bone in skeleton (this is actually always the root).
      */
      bone_iterator       begin()       { return bone_iterator(m_bones.begin());      }
      const_bone_iterator begin() const { return const_bone_iterator(m_bones.begin());}

      /**
      *
      * @return  An iterator to a position one past the last bone in the skeleton (this is one of perhaps several end-effctors).
      */
      bone_iterator       end()       { return bone_iterator(m_bones.end());      }
      const_bone_iterator end() const { return const_bone_iterator(m_bones.end());}

    public:

      /**
       * Get Root Bone.
       *
       * @return     A pointer to the root bone of the skeleton or null if none exist
       */
      bone_type * root() {return m_root; }

      /**
       * Get Root Bone.
       *
       * @return     A pointer to the root bone of the skeleton or null if none exist
       */
      bone_type const * root() const {return m_root; }

      /**
      * Skeleton Size.
      *
      * @return   The number of bones in the skeleton.
      */
      size_t size() const { return m_bones.size(); }

      /**
      * Get Bone.
      *
      * @param  number    The bone number of the sought bone.
      *
      * @return           A reference to the bone with the specified number.
      */
      bone_type * get_bone(size_t const & number) { return find_bone( number ); }
      
      /**
      * Get Bone.
      *
      * @param  number    The bone number of the sought bone.
      *
      * @return           A reference to the bone with the specified number.
      */
      bone_type const * get_bone(size_t const & number)  const { return find_bone( number ); }


      /**
      * Get Bone.
      *
      * @param  number    The bone number of the sought bone.
      *
      * @return           An iterator to the bone with the specified number.
      */
        bone_iterator get_bone_iterator(size_t const & number)
        {
          bone_iterator bone = this->begin();
          bone_iterator end  = this->end();
          for(;bone!=end;++bone)
          {
            if(bone->get_number() == number)
              return bone;
          }
          return end; 
        }

      /**
      * Get Bone.
      *
      * @param  name     The name of the sought bone.
      *
      * @return           A reference to the bone with the specified name.
      */
      bone_type * get_bone(std::string const & name)
      {
        lut_iterator lookup = m_bone_number_lut.find(name);
        if(lookup==m_bone_number_lut.end())
          return 0;
        size_t number = m_bone_number_lut[name];
        return get_bone( number );
      }

    public:

      /**
      * Bone Renaming Method.
      *
      * @param bone  a pointer to the bone that should be renamed.
      * @param name  The new name of the bone.
      *
      * @return      If the rename is succesfull then the return value is true otherwise it is false.
      */
      bool rename_bone( bone_type * bone, std::string const & name)
      {
        assert( bone || !"rename_bone(): Bone was null");
        lut_iterator entry = m_bone_number_lut.find(bone->get_name());

        if(entry!=m_bone_number_lut.end())
          m_bone_number_lut.erase(entry);

        BoneAccess::set_name( bone, name );

        m_bone_number_lut[bone->get_name()] = bone->get_number();

        return true;
      }

      /**
      * Create new bone.
      * This method should only be used when inserting the first bone into the skeleton.
      * This bone will automatically become the root bone.
      *
      * @return    A pointer to the newly added bone.
      */
      bone_type * create_bone()
      {
        assert(!m_root         || !"Skeleton::create_bone(): Sorry you forgot to specify a parent bone");
        assert(m_bones.empty() || !"Skeleton::create_bone(): Sorry you forgot to specify a parent bone");

        bone_type * bone = register_bone();
        m_root = bone;

        bone_type * const null = 0;

        BoneAccess::set_parent( bone, null );
        return bone;
      }

      /**
      * Add new bone.
      * This method should be used to add bones as children of already
      * added bones in the skeleton.
      *
      * @param parent    A pointer to the parent bone. Must already have
      *                  been added to the skeleton.
      * @return         A pointer to the newly added bone.
      */
      bone_type * create_bone(bone_type * parent)
      {
        assert(parent->skeleton()==this || !"Skeleton::create_bone(): Sorry specified parent bone did not belong to skeleton");

        bone_type * bone = register_bone();

        BoneAccess::add_child( parent, bone );
        BoneAccess::set_parent( bone, parent );
        return bone;
      }

      /**
      * Clear all content in skeleton.
      */
      void clear()
      {
        m_root = 0;
        m_bones.clear();
        m_bone_number_lut.clear();
        m_bone_ptr_lut.clear();
      }

    public:

      /**
      * Compute Pose of Skeleton.
      * This method computes the absolute bone transformation of each
      * bone and updates the bone space transformation of each bone.
      */
      void compute_pose()
      {
        bone_iterator end   = m_bones.end();
        bone_iterator bone  = m_bones.begin();
        for(;bone!=end;++bone)
        {
          if(bone->is_root())
          {
            bone->absolute() = bone->relative();
          }
          else
          {
            bone->absolute() = bone_traits::compute_absolute_pose_transform(bone->parent()->absolute(), bone->relative());
          }          
          bone->bone_space_transform() = bone_traits::compute_bone_space_transform( bone->absolute(),  bone->bone_space() );
        }
      }

      /**
      * Clear all relative transforms to the identity transformation
      * and sets the bone weights to zero. This method is usefull to
      * invoke prior to doing animation blending.
      */
      void clear_pose()
      {
        bone_iterator end  = m_bones.end();
        bone_iterator bone = m_bones.begin();
        for(;bone!=end;++bone)
        {
          bone->accumulated_weight() = value_traits::zero();
          bone->relative()           = bone_traits::get_identity_transform();
        }
      }

      /**
       * Thie method sets the relative bone transforms to the bind pose
       * and compute the absolute bone transformation of each bone. Also
       * all bone weights are set to zero.
       */
      void set_bind_pose()
      {
        bone_iterator end  = m_bones.end();
        bone_iterator bone = m_bones.begin();
        for(;bone!=end;++bone)
        {
          bone->accumulated_weight() = value_traits::zero();
          bone->relative() = bone->bind_pose();
        }
        this->compute_pose();
      }

      /**
       * This method sets all bone weights to zero.
       */
      void clear_weights()
      {
        bone_iterator end  = m_bones.end();
        bone_iterator bone = m_bones.begin();
        for(;bone!=end;++bone)
        {
          bone->accumulated_weight() = value_traits::zero();
        }
      }

    protected:

      /**
      * Find Bone.
      *
      * @param  number    The bone number of the sought bone.
      *
      * @return           A pointer to the bone with the specified number or null if it does not exist.
      */
      bone_type * find_bone(size_t const & number)  const
      {
        if(number<0)
          return 0;
        if(number >= size() )
          return 0;
        return m_bone_ptr_lut[number];
      }

      /**
      * Register new Bone in Skeleton.
      *
      * @return    A pointer to the newly inserted bone.
      */
      bone_type * register_bone()
      {
        m_bones.push_back(bone_type());
        bone_type * bone = &m_bones.back();

        assert(bone || !"Skeleont::register_bone(): Sorry something bad happen!");

        size_t number = static_cast<size_t>( size() ) - 1;

        BoneAccess::set_number( bone, number );
        BoneAccess::set_skeleton( bone, this );

        m_bone_number_lut[ bone->get_name() ] = bone->get_number();

        m_bone_ptr_lut.push_back(bone);

        return bone;
      }

    };

  } // namespace skeleton
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKELETON_SKELETON_H
#endif
