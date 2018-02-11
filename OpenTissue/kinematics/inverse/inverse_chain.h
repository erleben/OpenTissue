#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_CHAIN_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_CHAIN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/iterator/indirect_iterator.hpp>

#include <stdexcept>
#include <list>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Chain Class.
      * This class represents a kinematic chain in a skeleton.
      *
      * A kinematic chain is simply a connected sequence of bones. The
      * first bone in the sequence is named the root-bone and the last
      * bone is named the end-effector.
      *
      * Observe that the root of a kinematic chain does not need
      * to be the root of the skeleton. However, the root of the
      * chain does need to be the ``inboard'' bone and the end-effector
      * needs to the be the ``outboard'' bone.
      * 
      * By inboard we mean closer to the root of the skeleton and by
      * outboard we mean further away from the skeleton root.
      *
      * @tparam skeleton_type_      The type of skeleton that this kinematic
      *                             chain can be defined on.
      */
      template<typename skeleton_type_ >
      class Chain
      {
      public:

        typedef          skeleton_type_               skeleton_type;
        typedef typename skeleton_type::math_types    math_types;
        typedef typename skeleton_type::bone_type     bone_type;
        typedef typename skeleton_type::bone_traits   bone_traits;

        typedef typename math_types::real_type        real_type;
        typedef typename math_types::vector3_type     vector3_type;

      protected:

        typedef typename math_types::value_traits     value_traits;

        typedef std::list<bone_type*>                       bone_container;
        typedef typename bone_container::iterator           bone_ptr_iterator;
        typedef typename bone_container::reverse_iterator   reverse_bone_ptr_iterator;

      public:

        typedef boost::indirect_iterator<bone_ptr_iterator>               bone_iterator;
        typedef boost::indirect_iterator<reverse_bone_ptr_iterator>       reverse_bone_iterator;


      protected:

        bone_container m_bones;  ///< All the bones in the chain, stored sequentially from root to end-effector.

        bool m_only_position;  ///< Boolean flag indicating whether this chain goal is purely positional or if it includes orientation. Default is true.
     
        vector3_type m_p_local;  ///< In local end-effector coordinates
        vector3_type m_x_local;  ///< In local end-effector coordinates
        vector3_type m_y_local;  ///< In local end-effector coordinates

        vector3_type m_p_global;  ///< In global world (root) coordinates
        vector3_type m_x_global;  ///< In global world (root) coordinates
        vector3_type m_y_global;  ///< In global world (root) coordinates

        real_type m_weight_p; ///< The weighting of p goal. Default value is one.
        real_type m_weight_x; ///< The weighting of x goal. Default value is one.
        real_type m_weight_y; ///< The weighting of y goal. Default value is one.

      public:

        /**
         * Get Only Positional Flag.
         *
         * @return   A reference to a boolean flag. If set to true then
         *           the chain goal is only positional and otherwise both
         *           positional and orientation.
         */
        bool       & only_position()       { return m_only_position; }
        bool const & only_position() const { return m_only_position; }
        
        /**
        * Get Local transformation
        *
        * @return   A reference to the local translation  of the end effector
        **/
        vector3_type const & p_local() const { return m_p_local; }
        vector3_type       & p_local()       { return m_p_local; }
        
        /**
        * Get Local transformation
        *
        * @return   A reference to the local x axis of the end effector
        **/

        vector3_type const & x_local() const { return m_x_local; }
        vector3_type       & x_local()       { return m_x_local; }
        
        /**
        * Get Local transformation
        *
        * @return   A reference to the local y axis of the end effector
        **/

        vector3_type const & y_local() const { return m_y_local; }
        vector3_type       & y_local()       { return m_y_local; }
        
        /**
        * Get global transformation
        *
        * @return   A reference to the global translation  of the end effector
        *           i.e in WCS
        **/        
        vector3_type const & p_global() const { return m_p_global; }
        vector3_type       & p_global()       { return m_p_global; }

        /**
        * Get global transformation
        *
        * @return   A reference to the global xaxis  of the end effector
        *           i.e in WCS
        **/   
        vector3_type const & x_global() const { return m_x_global; }
        vector3_type       & x_global()       { return m_x_global; }

        /**
        * Get global transformation
        *
        * @return   A reference to the global yaxis  of the end effector
        *           i.e in WCS
        **/   
        vector3_type const & y_global() const { return m_y_global; }
        vector3_type       & y_global()       { return m_y_global; }

        /**
         * Get Partial Goal Weight.
         *
         * @return       The current goal weight.
         */
        real_type const & weight_p() const { return m_weight_p; }
        real_type const & weight_x() const { return m_weight_x; }
        real_type const & weight_y() const { return m_weight_y; }

        /**
         * Set Partial Goal Weight.
         *
         * @param value    A non-negative goal weight. Large values means that more importance should be applied to reach the corresponding goal.
         */
        void set_weight_p(real_type const & value){  this->m_weight_p = value >= value_traits::zero() ? value : value_traits::zero();  }
        void set_weight_x(real_type const & value){  this->m_weight_x = value >= value_traits::zero() ? value : value_traits::zero();  }
        void set_weight_y(real_type const & value){  this->m_weight_y = value >= value_traits::zero() ? value : value_traits::zero();  }

      public:

        /**
        * Comparison Operator.
        *
        * @param chain   Another chain to compare with.
        *
        * @return        If the other chain have the same root and end-effector then we define the two chains to be the same.
        */
        bool operator==(Chain const & chain)
        {
          if (this->m_bones.empty() || chain.m_bones.empty() )
            return false;

          if( this->m_bones.front() != chain.m_bones.front())
            return false;

          if( this->m_bones.back() == chain.m_bones.back() )
            return false;

          return true;
        }

      public:

        /**
        * Initialize Chain.
        * This method traverses all the bones inbetween the specified
        * end-effector and root. While traversing the bones they are
        * added to the chain.
        *
        * @param root_           A pointer to the root bone of the chain.
        * @param end_effector_   A pointer to the end-effector bone of the chain.
        **/
        void init(bone_type const * root_, bone_type const * end_effector_) 
        {
          // Strip away const qualifier
          bone_type * end_effector  = const_cast<bone_type*>(end_effector_);
          bone_type * root          = const_cast<bone_type*>( root_ );

          if(root==0)
            throw std::invalid_argument("root was null");
          if(end_effector==0)
            throw std::invalid_argument("end_effector was null");

          this->m_bones.push_front(end_effector);
          bone_type * bone = end_effector;
          while(bone!=root && bone!=0)
          {
            bone_type * parent = const_cast<bone_type*>( bone->parent() );
            this->m_bones.push_front( parent);
            bone = parent;
          }

          if(bone==0)
            throw std::logic_error("root was not an ancestor of end-effector");

          // Setup default goal positions and weights

          this->m_weight_p = value_traits::one();
          this->m_weight_x = value_traits::one();
          this->m_weight_y = value_traits::one();

          this->m_p_local = vector3_type( value_traits::zero(), value_traits::zero(), value_traits::zero());
          this->m_x_local = vector3_type(  value_traits::one(), value_traits::zero(), value_traits::zero());
          this->m_y_local = vector3_type( value_traits::zero(),  value_traits::one(), value_traits::zero());

          typename math_types::coordsys_type goal = bone_traits::convert( end_effector->absolute() );
          typename math_types::quaternion_type Q = goal.Q();
          typename math_types::matrix3x3_type M;

          M = Q;

          this->m_p_global = goal.T();
          
          this->m_x_global = M.column(0);
          this->m_y_global = M.column(1);

          this->m_only_position = true;
        }

        /**
        * Get Bone Iterator. 
        *
        * @return   A bone iterator to the first bone in the chain.
        */
        bone_iterator         bone_begin()    {  return bone_iterator(  this->m_bones.begin() );       }

        /**
        * Get End Bone Iterator. 
        *
        * @return   A bone iterator to the position one past the last bone (i.e. the end-effector) in the chain.
        */
        bone_iterator         bone_end()      {  return bone_iterator(  this->m_bones.end() );         }

        /**
        * Get Reverse Bone Iterator. 
        *
        * @return   A reverse bone iterator to the last
        *           bone in the chain (ie. the end-effector).
        */
        reverse_bone_iterator rbone_begin()   {  return reverse_bone_iterator(this->m_bones.rbegin()); }

        /**
        * Get Reverse End Bone Iterator.
        *
        * @return   A reverse bone iterator to the position just prior to
        *           the first bone in the chain (i.e the root).
        */
        reverse_bone_iterator rbone_end()     {  return reverse_bone_iterator(this->m_bones.rend());   }

        /**
        * Get Root Bone.
        *
        * @Return   A pointer to the root bone of the chain or null
        *          if the chain is not initialised.
        */
        bone_type * const get_root()         const 
        {
          if(this->m_bones.empty())
            return 0;
          return *(this->m_bones.begin());
        }

        /**
        * Get end-effector Bone.
        *
        * @return   A pointer to the end-effector bone
        *           or null if the chain is un-initialized.
        */
        bone_type * const get_end_effector() const 
        {  
          if(this->m_bones.empty())
            return 0;
          return *(this->m_bones.rbegin());
        }

        /**
        * Get Dimension of Goal.
        *
        * @return     The dimension of the goal placement. If only positional then the dimension is 3. If both position and orientation is specified then the dimension is 9.
        */
        size_t get_goal_dimension() const   
        {  
          return (this->m_only_position?3u:9u); 
        }
        
      };

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_CHAIN_H
#endif
