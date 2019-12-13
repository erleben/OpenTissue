#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_SEPARATED_COLLISION_CONTACT_FIXED_STEP_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_SEPARATED_COLLISION_CONTACT_FIXED_STEP_SIMULATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_simulator_interface.h>
#include <OpenTissue/dynamics/mbd/mbd_stack_propagation.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Separated Collision and Contact Time Integration.
    *
    * The implementation herein is a generalization of the simulator described in:
    *
    *            "Nonconvex Rigid Bodies with Stacking" by Guendelman, Bridson, and Fedkiw,
    *            SIGGRAPH 2003, ACM TOG 22, 871-878 (2003).
    *
    * In the paper a simple collision resolver is used, which simply iterates one-time
    * through all contacts in increasing order of penetration depth. We have generalized
    * our implementation such that the end-user can supply any kind of collision-resolving
    * scheme, also any kind of collision law can be used.
    *
    * In the paper it is suggested to re-evaluate contacts, i.e. run the collision detection,
    * after each resolved collision. Our implementation uses the more aggressive approach
    * proposed in the paper, which re-run the collision detection for each iteration over
    * all the contact points.
    *
    * The contact graph is also only builded once and not re-evaluated during simulation,
    * the contact graph building algorithm is given implicitly through the stack propagation
    * tool. Our contact graph and stack-layer detection algorithms are different from the ones
    * desribed in the paper.
    *
    * Finally we have chosen to use a semi-implicit approach for collision resolving,
    * contact handling and shock propagation.
    *
    * The simulator was tested using the IterateOnceCollisionReslover together
    * with the FrictionalNewtonCollisionLaw.
    *
    *
    * IMPORTANT: This simulator can not be used together with a stepper that also
    * uses StackPropagation. The reasons for this is that such a stepper
    * will also extend the node_traits with algorithm specific traits from the
    * class StackPropagation. This simulator will do the same thing causing
    * a ambuigity in base classes due to multiple inheritence.
    *
    */
    template< typename mbd_types  >
    class SeparatedCollisionContactFixedStepSimulator  
      : public SimulatorInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy          math_policy;
      typedef typename math_policy::index_type           size_type;
      typedef typename math_policy::real_type            real_type;
      typedef typename math_policy::vector3_type         vector3_type;
      typedef typename math_policy::idx_vector_type      idx_vector_type;
      typedef typename math_policy::matrix_type          matrix_type;
      typedef typename math_policy::vector_type          vector_type;
      typedef typename math_policy::value_traits         value_traits;
      
      typedef typename mbd_types::group_type                       group_type;
      typedef typename mbd_types::group_ptr_container              group_ptr_container;
      typedef typename mbd_types::stepper_policy                   stepper_policy;
      typedef          StackPropagation<mbd_types>            propagation_algorithm;
      typedef typename mbd_types::material_library_type            material_library_type;
      typedef typename material_library_type::material_iterator      material_iterator;

    public:

      class node_traits 
        : public propagation_algorithm::node_traits 
      {};

      class edge_traits 
        : public propagation_algorithm::edge_traits 
      {};

      class constraint_traits 
        : public propagation_algorithm::constraint_traits 
      {};

    protected:

      /**
      * A Stepper Functor.
      * This functor is used to pass the stepper algorithm
      * on to the stack propagation algorithm.
      */
      struct StepperFunctor
      {
        stepper_policy * m_stepper;        ///< The stepper algorithm to be used.
        real_type        m_h;              ///< The time step to be used.

        void operator()(group_type & layer)
        {
          assert(m_stepper || !"StepperFunc::operator(): missing stepper");
          m_stepper->run(layer,m_h);
        }
      };

    protected:

      propagation_algorithm      m_propagation;          ///< Stack Propagation Algorithm to be used.
      StepperFunctor             m_stepper_functor;      ///< The stepper functor to use on each layer.
      vector_type                m_s_cur;                ///< Generalized position vector of all bodies.
      vector_type                m_s_predicted;          ///< Generalized position vector of all bodies.
      vector_type                m_u;                    ///< Generalized velocity vector of all bodies.
      vector_type                m_F;                    ///< Generalized force vector of all bodies.
      matrix_type                m_invM;                 ///< Generalized inverse mass matrix of all bodies.
      vector_type                m_restitution;          ///< Temporary storage for keeping restitution values.
      group_type *               m_all;                  ///< A pointer to a group containing all bodies in the configuration.
      group_ptr_container        m_groups;               ///< Temporary Storage, used to hold results from the collision
                                                         ///< detection engine. 
    public:

      SeparatedCollisionContactFixedStepSimulator(){}

      virtual ~SeparatedCollisionContactFixedStepSimulator(){}

    public:

      void run(real_type const & time_step)
      {
        assert(time_step>0 || !"SeparatedCollisionContactFixedStepSimulator::run(): time step must be positive");

        m_all = this->get_configuration()->get_all_body_group();

        size_type n = m_all->size_bodies();
        
        math_policy::resize( m_s_cur, 7*n);
        math_policy::resize( m_s_predicted, 7*n);
        math_policy::resize( m_u, 6*n);
        math_policy::resize( m_F, 6*n);

        mbd::get_position_vector(*m_all, m_s_cur);
        mbd::get_inverse_mass_matrix(*m_all,m_invM);

        mbd::compute_scripted_motions(*m_all, this->time() + time_step);

        m_stepper_functor.m_stepper = this->get_stepper();
        m_stepper_functor.m_h = time_step;

        int collision_iterations = 5;
        for(int i=0;i<collision_iterations;++i)
          resolve_collisions(time_step);
        velocity_update(time_step);

        extract_restitution();
        int contact_iterations = 10;
        real_type amount = 1./contact_iterations;
        for(int i=0;i<contact_iterations;++i)
        {
          increase_restitution(amount);
          contact_handling(time_step);
        }
        int shock_iterations = 1;
        for(int i=0;i<shock_iterations;++i)
          shock_propagation(time_step);
        restore_restitution();

        position_update(time_step);
        SimulatorInterface<mbd_types>::update_time(time_step);
      }

    protected:

      void velocity_update(real_type const & h)
      {
        assert(m_all || !"SeparatedCollisionContactFixedStepSimulator::velocity_update(): missing all group");
        mbd::get_external_force_vector(*m_all,m_F,true);
        mbd::get_velocity_vector(*m_all, m_u);

        //m_u += ublas::prod(m_invM, m_F)*h;
        math_policy::prod_add(m_invM, m_F,m_u,h);
        
        mbd::set_velocity_vector(*m_all,m_u);
      }

      void position_update(real_type const & h)
      {
        assert(m_all || !"SeparatedCollisionContactFixedStepSimulator::position_update(): missing all group");
        mbd::get_velocity_vector(*m_all, m_u);
        mbd::compute_position_update(*m_all,m_s_cur,m_u,h,m_s_cur);
        mbd::set_position_vector(*m_all,m_s_cur);
      }

      void resolve_collisions(real_type const & h)
      {
        assert(m_all || !"SeparatedCollisionContactFixedStepSimulator::resolve_collisions(): missing all group");

        //--- use predicted postion x' = x + h*(v+h*F)
        mbd::get_external_force_vector(*m_all,m_F, true);
        mbd::get_velocity_vector(*m_all, m_u);

        m_u += prod(m_invM, m_F)*h;

        mbd::compute_position_update(*m_all,m_s_cur,m_u,h,m_s_predicted);
        mbd::set_position_vector(*m_all,m_s_predicted);

        this->get_collision_detection()->run( m_groups );
        for(typename group_ptr_container::iterator tmp=m_groups.begin();tmp!=m_groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_stepper()->resolve_collisions(*group);
        }
      }

      void contact_handling(real_type const & h)
      {
        assert(m_all || !"SeparatedCollisionContactFixedStepSimulator::contact_handling(): missing all group");
        //--- use predicted postion x' = x + h*v'
        mbd::get_velocity_vector(*m_all, m_u);
        mbd::compute_position_update(*m_all,m_s_cur,m_u,h,m_s_predicted);
        mbd::set_position_vector(*m_all,m_s_predicted);

        this->get_collision_detection()->run( m_groups );
        for(typename group_ptr_container::iterator tmp=m_groups.begin();tmp!=m_groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_stepper()->resolve_collisions(*group);
        }
      }

      void shock_propagation(real_type const & h)
      {
        assert(m_all || !"SeparatedCollisionContactFixedStepSimulator::shock_propagation(): missing all group");

        //--- use predicted postion x' = x + h*v'
        mbd::get_velocity_vector(*m_all, m_u);
        mbd::compute_position_update(*m_all,m_s_cur,m_u,h,m_s_predicted);
        mbd::set_position_vector(*m_all,m_s_predicted);

        this->get_collision_detection()->run( m_groups );
        for(typename group_ptr_container::iterator tmp=m_groups.begin();tmp!=m_groups.end();++tmp)
        {
          group_type * group = (*tmp);
          m_propagation.run( 
              *group
            , m_stepper_functor
            , typename propagation_algorithm::fixate_tag()
            , typename propagation_algorithm::upward_tag()  
          );
        }
      }

      void extract_restitution()
      {
        size_type N = this->get_configuration()->get_material_library()->size_materials();

        math_policy::resize( m_restitution, N);
        
        typename vector_type::iterator e_n = m_restitution.begin();

        material_iterator material = this->get_configuration()->get_material_library()->material_begin();
        material_iterator end      = this->get_configuration()->get_material_library()->material_end();
      
        for(;material!=end;++material)
        {
          *e_n++ = material->normal_restitution();
          material->normal_restitution() = -1.;
        }
      }

      void increase_restitution(real_type const & amount)
      {
        using std::min;

        material_iterator material = this->get_configuration()->get_material_library()->material_begin();
        material_iterator end      = this->get_configuration()->get_material_library()->material_end();

        for(;material!=end;++material)
        {
          real_type e_n = material->normal_restitution() + amount;
          e_n = min(value_traits::zero(),e_n);
          material->normal_restitution() = e_n;
        }
      }

      void restore_restitution()
      {
        typename vector_type::iterator e_n = m_restitution.begin();

        material_iterator material = this->get_configuration()->get_material_library()->material_begin();
        material_iterator end      = this->get_configuration()->get_material_library()->material_end();

        for(;material!=end;++material)
        {
          material->normal_restitution() = (*e_n++);
        }
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_SEPARATED_COLLISION_CONTACT_FIXED_STEP_SIMULATOR_H
#endif
