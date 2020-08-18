#ifndef	OPENTISSUE_DYNAMICS_MBD_MBD_TYPES_H
#define	OPENTISSUE_DYNAMICS_MBD_MBD_TYPES_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>


#include <OpenTissue/collision/collision_geometry_interface.h>


#include <OpenTissue/utility/utility_identifier.h>

#include <OpenTissue/dynamics/mbd/mbd_body_group.h>
#include <OpenTissue/dynamics/mbd/mbd_material.h>
#include <OpenTissue/dynamics/mbd/mbd_material_library.h>
#include <OpenTissue/dynamics/mbd/mbd_contact_point.h>
#include <OpenTissue/dynamics/mbd/mbd_body.h>
#include <OpenTissue/dynamics/mbd/mbd_edge.h>
#include <OpenTissue/dynamics/mbd/mbd_configuration.h>
#include <OpenTissue/dynamics/mbd/mbd_joint_socket.h>

#include <OpenTissue/dynamics/mbd/mbd_collision_info.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_constraint_interface.h>
#include <OpenTissue/dynamics/mbd/interfaces/mbd_sub_constraint_interface.h>
#include <OpenTissue/dynamics/mbd/interfaces/mbd_force_interface.h>
#include <OpenTissue/dynamics/mbd/interfaces/mbd_joint_interface.h>
#include <OpenTissue/dynamics/mbd/interfaces/mbd_scripted_motion_interface.h>

#include <boost/iterator/indirect_iterator.hpp>

#include <list>
#include <vector>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    *  Type Binder Class.
    * This class binds together all template classes needed by the 
    * Multibody Engine. You should use the type definitions herein to
    * instanciate you simulator and simulator objects.
    */
    template<
      typename math_policy_  ///< Math types, precision, vector3, quaternion, large scale matrix library types etc..
      , template< typename > class sleepy_policy_               
      , template< typename > class stepper_policy_              
      , template< typename > class collision_detection_policy_
      , template< typename > class simulator_type_
    >
    class Types
    {
    public:

      typedef Types<math_policy_,sleepy_policy_,stepper_policy_,collision_detection_policy_,simulator_type_> types;

      typedef math_policy_                                          math_policy;
      typedef OpenTissue::utility::Identifier                       identifier_type;
      typedef Body<types>                                           body_type;
      typedef Edge<types>                                           edge_type;
      typedef Configuration<types>                                  configuration_type;
      typedef ScriptedMotionInterface<types>                        scripted_motion_type;
      typedef ForceInterface<types>                                 force_type;
      typedef BodyGroup<types>                                      group_type;
      typedef Material<types>                                       material_type;
      typedef MaterialLibrary<types>                                material_library_type;

      typedef OpenTissue::collision::GeometryInterface<math_policy> geometry_type;
      typedef CollisionInfo<types>                                  collision_info_type;

      typedef ConstraintInterface<types>          constraint_type;
      typedef SubConstraintInterface<types>       sub_constraint_type;
      typedef ContactPoint<types>                 contact_type;
      typedef JointInterface<types>               joint_type;
      typedef JointSocket<types>                  socket_type;

      typedef typename std::vector<group_type>                                                            group_container;

      typedef typename std::list<group_type*>                                                             group_ptr_container;
      typedef boost::indirect_iterator<typename group_ptr_container::iterator,group_type>                 indirect_group_iterator;
      typedef boost::indirect_iterator<typename group_ptr_container::const_iterator,group_type>           const_indirect_group_iterator;

      typedef typename std::list<contact_type*>                                                           contact_ptr_container;
      typedef boost::indirect_iterator<typename contact_ptr_container::iterator,contact_type>             indirect_contact_iterator;
      typedef boost::indirect_iterator<typename contact_ptr_container::const_iterator,contact_type>       const_indirect_contact_iterator;

      typedef typename std::vector<contact_type>                                                          contact_container;
      typedef typename contact_container::iterator                                                        contact_iterator;
      typedef typename contact_container::const_iterator                                                  const_contact_iterator;

      typedef typename std::list<edge_type *>                                                             edge_ptr_container;
      typedef boost::indirect_iterator<typename edge_ptr_container::iterator,edge_type>                   indirect_edge_iterator;
      typedef boost::indirect_iterator<typename edge_ptr_container::const_iterator,edge_type>             const_edge_iterator;

      typedef typename std::list<joint_type *>                                                            joint_ptr_container;
      typedef boost::indirect_iterator<typename joint_ptr_container::iterator,joint_type>                 indirect_joint_iterator;
      typedef boost::indirect_iterator<typename joint_ptr_container::const_iterator,joint_type>           const_indirect_joint_iterator;

      typedef typename std::list<force_type *>                                                            force_ptr_container;
      typedef boost::indirect_iterator<typename force_ptr_container::iterator,force_type>                 indirect_force_iterator;
      typedef boost::indirect_iterator<typename force_ptr_container::const_iterator,force_type>           const_indirect_force_iterator;

      typedef typename std::list<body_type*>                                                              body_ptr_container;
      typedef boost::indirect_iterator<typename body_ptr_container::iterator,body_type>                   indirect_body_iterator;
      typedef boost::indirect_iterator<typename body_ptr_container::const_iterator,body_type>             const_indirect_body_iterator;

      typedef typename std::list<constraint_type* >                                                       constraint_ptr_container;
      typedef boost::indirect_iterator<typename constraint_ptr_container::iterator,constraint_type>       indirect_constraint_iterator;
      typedef boost::indirect_iterator<typename constraint_ptr_container::const_iterator,constraint_type> const_indirect_constraint_iterator;

      typedef sleepy_policy_<types>               sleepy_policy;
      typedef stepper_policy_<types>              stepper_policy;
      typedef collision_detection_policy_<types>  collision_detection_policy;
      typedef simulator_type_<types>              simulator_type;    

    protected:

      class NodeTraitsClass 
        : public sleepy_policy::node_traits
        , public stepper_policy::node_traits
        , public collision_detection_policy::node_traits
        , public simulator_type::node_traits
      {            
      };

      class EdgeTraitsClass 
        : public sleepy_policy::edge_traits
        , public stepper_policy::edge_traits
        , public collision_detection_policy::edge_traits
        , public simulator_type::edge_traits
      {};

      class ConstraintTraitsClass 
        : public sleepy_policy::constraint_traits
        , public stepper_policy::constraint_traits
        , public collision_detection_policy::constraint_traits
        , public simulator_type::constraint_traits
      {};   

    public:
      typedef EdgeTraitsClass                 edge_traits;
      typedef NodeTraitsClass                 node_traits;
      typedef ConstraintTraitsClass           constraint_traits;           
    };

  } //End of namespace mbd

} //End of namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_MBD_TYPES_H
#endif
