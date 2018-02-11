#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_SINUS_CONE_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_SINUS_CONE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <cassert>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
      namespace sinus_cone
      {
        
        /**
         * Project sub-vector onto Sinus cone  of a Bone.
         *
         * @see BoneTraits::joint_limit_projection() for more details.
         */
        template<typename bone_type, typename vector_range>
        inline void joint_limit_projection( bone_type const & bone, vector_range & sub_theta)
        {
          using std::max;
          using std::min;
          
          assert(sub_theta.size() == bone.active_dofs() || !"joint_limit_projection() invalid dimension");
          assert(sub_theta.size() == bone.active_dofs() || !"joint_limit_projection() invalid dimension");

          // .... ToDo add implementatio here ....
        }
                
      } // namespace sinus_cone
      
      /**
       * Sinus Cone Class.
       * This class holds the data necessary for representing a sinus cone. The class is no-more than a
       * pure data-class. All algorithms and functionality can be found in the sinus_cone namespace.
       */
      template<typename bone_traits >
      class SinusCone 
        {
        public:
          
          typedef typename bone_traits::math_types::real_type        real_type;
          typedef typename bone_traits::math_types::value_traits     value_traits;
          
        protected:
          
          // ... ToDo Add Implementation ...
          
        public:
          
          SinusCone()
          {
            // ... ToDo Add Implementation ...
          }
          
          
          // ... ToDo Add Implementation ...
          
        };
      
    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_SINUS_CONE_H
#endif
