#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_PROPAGATION_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_PROPAGATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/mbd_stack_analysis.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Stack Propagation.
    * Apply some algorithm in a stack bottom-up fashion. Thus
    * the effects of the algorithm are propagated from the bottom
    * of a stack to the top.
    *
    * This is in some way similar to make a pre-ordring of the constraint
    * variables for an iterative method, however there is one major difference
    * from simple pre-ordering. The difference is that objects are fixiated
    * during the stack propagration. This can be compared to constraint-variables
    * only have an effect in one direction, the upward direction. In terms of
    * iterative methods, it is a pre-ordering of variables and restriction to
    * a preferred search direction for solutions.
    */
    template<typename mbd_types >
    class StackPropagation : public StackAnalysis<mbd_types>
    {
    public:

      typedef StackAnalysis<mbd_types>                  analysis_type;

      typedef typename mbd_types::math_policy::index_type    size_type;
      typedef typename mbd_types::math_policy::index_type    index_type;

      typedef typename mbd_types::math_policy::real_type     real_type;
      typedef typename mbd_types::math_policy::vector3_type  vector3_type;
      typedef typename mbd_types::group_type                 group_type;
      typedef typename mbd_types::body_type                body_type;
      typedef typename mbd_types::edge_type                edge_type;

      typedef typename mbd_types::contact_type             contact_type;
      typedef typename mbd_types::material_type            material_type;
      typedef typename mbd_types::edge_ptr_container       edge_ptr_container;
      typedef typename mbd_types::indirect_edge_iterator            indirect_edge_iterator;
      typedef typename mbd_types::group_container          group_container;

    public:

      class node_traits 
        : public analysis_type::node_traits
      {
      public:
        bool m_sp_fixiated;   ///< Boolean flag used to remember if a body was temporarily turned fixed.
      };

      class edge_traits 
        : public analysis_type::edge_traits
      {};

      class constraint_traits 
        : public analysis_type::constraint_traits
      {};

    protected:

      group_container m_layers;     ///< Storage for keeping stack layers.
      size_type       m_cnt;        ///< Number of layers in stack.

    public:

      struct upward_tag {};
      struct downward_tag {};
      struct fixate_tag {};

    public:


      /**
       * Get Number of Layers.
       * Note: only valid immediately after invocation of a run method. Re-runs do
       * not alter this value, only invocation of run methods may alter this value.
       *
       * @return   The number of stack layers.
       */
      size_type size() const { return m_cnt; }

      /**
      * Run Stack Propagation Algorithm.
      *
      * @param group        The total group upon which to perform stack propagation.
      * @param algorithm    An unary function that takes a reference to a body group type as an
      *                     argument. The argument represents a stack layer, where the bottom-most
      *                     bodies are fixiated. The end user can run any kind of algorithm on this
      *                     stack-layer.
      * @param fixate_tag   Tag dispatching, indicating that bottom-most objects should be fixiated before applying algorithm.
      * @param upward_tag   Tag dispacthing, indicating that shock moves from bottom to top.
      */
      template <typename algorithm_type>
      void run(group_type & group, algorithm_type & algorithm, fixate_tag, upward_tag )
      {
        m_cnt = StackAnalysis<mbd_types>::analyze(group,m_layers);
        for(index_type height=0;height<m_cnt;++height)
        {
          fixiate(height,m_layers[height]);
          algorithm(m_layers[height]);
          unfixiate(m_layers[height]);
        }
      }

      /*
      * @param fixate_tag   Tag dispatching, indicating that bottom-most objects should be fixiated before applying algorithm.
      * @param upward_tag   Tag dispacthing, indicating that shock moves from bottom to top.
      */
      template <typename algorithm_type>
      void rerun(group_type & /*group*/, algorithm_type & algorithm,  fixate_tag, upward_tag)
      {
        for(index_type height=0;height<m_cnt;++height)
        {
          fixiate(height,m_layers[height]);
          algorithm(m_layers[height]);
          unfixiate(m_layers[height]);
        }
      }

    public:

      template <typename algorithm_type>
      void run(group_type & group, algorithm_type & algorithm, upward_tag)
      {
        m_cnt = analyze(group,m_layers);
        for(index_type height=0;height<m_cnt;++height)
          algorithm(m_layers[height]);
      }

      template <typename algorithm_type>
      void rerun(group_type & group, algorithm_type & algorithm, upward_tag)
      {
        for(index_type height=0;height<m_cnt;++height)
          algorithm(m_layers[height]);
      }

      template <typename algorithm_type>
      void run(group_type & group, algorithm_type & algorithm, downward_tag )
      {
        assert(m_cnt >= 1 || !"StackPropagation::run(...,downward): Need at least one layer");

        m_cnt = StackAnalysis<mbd_types>::analyze(group,m_layers);
        index_type height=m_cnt-1;
        for(index_type layer=0;layer<m_cnt;++layer,--height)
        {
          algorithm(m_layers[height]);
        }
      }

      template <typename algorithm_type>
      void rerun(group_type & group, algorithm_type & algorithm, downward_tag)
      {
        assert(m_cnt >= 1 || !"StackPropagation::rerun(...,downward): Need at least one layer");

        index_type height=m_cnt-1;
        for(index_type layer=0;layer<m_cnt;++layer,--height)
          algorithm(m_layers[height]);
      }

    protected:

      /**
      * Fixiate all bottom bodies in a stack layer.
      *
      * @param height   The height of the layer.
      * @param layer    The stack layer.
      */
      void fixiate(index_type height, group_type & layer)
      {
        typename group_type::indirect_body_iterator begin = layer.body_begin();
        typename group_type::indirect_body_iterator end = layer.body_end();
        for(typename group_type::indirect_body_iterator body = begin;body!=end;++body)
        {
          if(body->m_sa_stack_height==height && !body->is_fixed())
          {
            body->m_sp_fixiated = true;
            body->set_fixed(true);
          }
          else
          {
            body->m_sp_fixiated = false;
          }
        }
      }

      /**
      * Un-fixiate all bottom bodies in a stack layer.
      *
      * This method assumes that the fixiate() method has been invoked prior.
      *
      * @param layer   The stack layer.
      */
      void unfixiate(group_type & layer)
      {
        typename group_type::indirect_body_iterator begin = layer.body_begin();
        typename group_type::indirect_body_iterator end = layer.body_end();
        for(typename group_type::indirect_body_iterator body = begin;body!=end;++body)
        {
          if(body->m_sp_fixiated)
            body->set_fixed(false);
        }
      }

    };


  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_PROPAGATION_H
#endif
