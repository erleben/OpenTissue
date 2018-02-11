#ifndef OPENTISSUE_KINEMATICS_SKELETON_BONE_H
#define OPENTISSUE_KINEMATICS_SKELETON_BONE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/iterator/indirect_iterator.hpp>

#include <list>
#include <cassert>

namespace OpenTissue
{
  namespace skeleton
  {

    /**
    * A Skeleton Bone.
    * Motion of skeleton bones is done by changing their relative
    * coordinate transform. During animation the absolute transform
    * of the bone is computed and the absolute bone space transform.
    *
    * The initial value of the relative transform is termed the bind
    * pose and it is considered to be a constant. Likewise is the bone
    * space considered to be a constant.
    */
    template<typename types>
    class Bone 
      : public types::bone_traits
    {
    public:

      friend class BoneAccess;  ///< A backdoor into the bone class. Used by skeleton class to manipulate private data in the bone class.

    public:

      typedef typename types::math_types                              math_types;
      typedef typename types::bone_traits                             bone_traits;
      typedef typename bone_traits::transform_type                    transform_type;

    protected:

      typedef typename math_types::value_traits                       value_traits;
      typedef typename types::math_types::coordsys_type               coordsys_type;
      typedef typename types::math_types::real_type                   real_type;

    public:

      typedef typename types::bone_type                               bone_type;
      typedef typename types::skeleton_type                           skeleton_type;
      typedef std::list<bone_type*>                                   bone_ptr_container;
      typedef typename bone_ptr_container::iterator                   bone_ptr_iterator;
      typedef boost::indirect_iterator<bone_ptr_iterator,bone_type>   bone_iterator;

    protected:

      std::string              m_name;
      size_t                   m_number;
      skeleton_type *          m_skeleton;
      bone_type *              m_parent;
      bone_ptr_container       m_children;

      transform_type           m_bind_pose;     ///< Initial placement of joint frame wrt. parent (given in parent frame)
      transform_type           m_bone_space;    ///< placement of bone frame wrt. joint frame (given in joint frame)
      transform_type           m_absolute;      ///< placement of joint frame wrt. parent (given in wcs frame)
      transform_type           m_transform;     ///< Bone space transform, i.e absolute placement of bone space in root frame.
      transform_type           m_relative;      ///< placement of joint frame wrt. parent (given in parent frame).

      real_type                m_accumulated_weight;  ///< Placeholder for accumulated blend weights (see Animation.blend_pose() and BlendScheduler.compute_pose() for details).

    public:

      Bone()
        : m_name("bone")
        , m_number(0)
        , m_parent(0)
        , m_accumulated_weight( value_traits::zero() )
      {}

    public:

      real_type const & accumulated_weight() const { return m_accumulated_weight; } 
      real_type       & accumulated_weight()       { return m_accumulated_weight; } 

      transform_type const & relative() const { return m_relative; }
      transform_type       & relative()       { return m_relative; }

      transform_type const & bind_pose() const {  return m_bind_pose;}
      transform_type       & bind_pose()       {  return m_bind_pose;}

      transform_type const & bone_space() const { return m_bone_space;}
      transform_type       & bone_space()       { return m_bone_space;}

      transform_type const & absolute() const { return m_absolute; }
      transform_type       & absolute()       { return m_absolute; }

      transform_type const & bone_space_transform() const { return m_transform; }
      transform_type       & bone_space_transform()       { return m_transform; }

    public:

      /**
      * Root Bone Query.
      *
      * @return   If this bone is the root bone then the return value
      *           is true otherwise it is false.
      */
      bool is_root() const { return !m_parent; }

      /**
      * Set bone name.
      *
      * @param name    The new name of the bone.
      *
      * @return        If the bone name was set succesfully then the return
      *                value is true otherwise it is false.
      */
      bool set_name( std::string const & name )
      {
        if( m_skeleton )
          return m_skeleton->rename_bone( this, name );
        return false;
      }

      /**
      * Get bone name.
      *
      * @return   A string representation of the bone name.
      */
      std::string  const & get_name()   const { return m_name; }

      /**
       * Get Bone Number
       *
       * @return The bone number.
       */
      size_t const & get_number() const { return m_number; }

      /**
       * Get the parent pointer.
       *
       * @return   The parent pointer.
       */
      bone_type const * parent()     const { return m_parent; }

      /**
       * Get Skeleton pointer.
       *
       * @return   The skeleton pointer.
       */
      skeleton_type    const * skeleton()     const { return m_skeleton; }


      /**
       * Test if this bone has any children. That is if it is a leaf bone.
       *
       * @return  If bone is a leaf then return value is true otherwise it is false.
       */
      bool is_leaf() const {  return m_children.empty(); }
	   /**
       * Test if this bone has more than one children. That is if it is a branch.
       *
       * @return  If bone is a leaf then return value is true otherwise it is false.
       */
	  bool is_branch() const { return (m_children.size()>1); }
	  bone_ptr_iterator child() { return m_children.begin(); };
	  int children() { return m_children.size(); }
    };

  } // namespace skeleton
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKELETON_BONE_H
#endif
