#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_KINETIC_ENERGY_SLEEPY_POLICY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_KINETIC_ENERGY_SLEEPY_POLICY_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Kinetic Energy Sleepy Policy.
    * This class implements a strategy for turning bodies sleepy, it is
    * based on tracking the kinetic energy. Both an absolute and a relative
    * test is applied.
    */
    template< typename mbd_types>
    class KineticEnergySleepyPolicy
    {
    protected:

      typedef typename mbd_types::math_policy::real_type     real_type;
      typedef typename mbd_types::math_policy::value_traits  value_traits;
      typedef typename mbd_types::math_policy::vector3_type  vector3_type;
      typedef typename mbd_types::group_type                 group_type;
      typedef typename mbd_types::body_type                  body_type;
      typedef typename group_type::indirect_body_iterator      indirect_body_iterator;

    protected:

      int m_max_entries;        ///< Maximum entries used in the kinetic energy array.

    public:

      class node_traits
      {
      public:

        real_type m_ksp_energy[5];       ///< A cyclic array containing the kinetic energy evaluations.
        int m_ksp_energy_idx;            ///< A cyclic index into the sleepy array

      public:

        node_traits()
          : m_ksp_energy_idx(0)
        { 
          //--- Initialize kinetic energy to some absurd high value
          for(int i=0;i<5;++i)
            m_ksp_energy[i]= value_traits::infinity();
        }

      };

      class edge_traits{};
      class constraint_traits{};

    public:

      KineticEnergySleepyPolicy()
        : m_max_entries(5)
      {}

      /**
      * Evaluate Sleepy State.
      * This method evaluates the sleppy state of a single body.
      *
      * @param body    A pointer to the body that should be evaluated. 
      */
      void evaluate(body_type * body)
      {
        //--- Fixed bodies must trivially be sleepy, so there is no need to perform any calculations
        if(body->is_fixed())
        {
          body->set_sleepy(true);
          return;
        }

        //      bool was_sleepy = body->is_sleepy();
        //--- Compute kinetic energy and store it in the node traits
        body->m_ksp_energy[body->m_ksp_energy_idx] = body->compute_kinetic_energy();
        int last_idx = body->m_ksp_energy_idx;
        body->m_ksp_energy_idx = (body->m_ksp_energy_idx + 1)%m_max_entries;

        //--- Loop over the kinetic energy stored in the last
        //--- m_max_entries iterations, and test that the energy in
        //--- each iteration is below a threshold. This is an absolute test.
        real_type threshold = body->get_mass()*0.00001;
        //real_type threshold = body->get_mass()*0.004802;
        for(int i=0;i<m_max_entries;++i)
        {
          int cur  = (last_idx-i)%m_max_entries;
          if(cur<0) cur += m_max_entries;
          int prev = (cur-1)%m_max_entries;
          if(prev<0) prev += m_max_entries;
          if(body->m_ksp_energy[cur] > threshold )
          {
            body->set_sleepy(false);
            return;
          }
        }

        //--- Loop over the kinetic energies one more time, this time
        //--- computing the energy change over the last m_max_entries
        real_type dE = value_traits::zero();
        for(int i=0;i<m_max_entries;++i)
        {
          int cur  = (last_idx-i)%m_max_entries;
          if(cur<0) cur += m_max_entries;
          int prev = (cur-1)%m_max_entries;
          if(prev<0) prev += m_max_entries;
          dE +=  body->m_ksp_energy[cur] - body->m_ksp_energy[prev];
        }
        //if(was_sleepy && 2.*dE < threshold)
        //{
        //  body->set_sleepy(true);
        //  return;
        //}
        //--- If the energy is decreasing then we say that the body
        //--- is sleepy, this is the relative test.
        real_type slack = 0.0001*threshold;
        if(dE>slack)
          body->set_sleepy(false);
        else
          body->set_sleepy(true);
      }

      /**
      * Evaluate Sleepy State.
      * Evaluates the sleepy state of all bodies in the specified range.
      * This method is invoked by the simulation method, see for instance
      * the FixedExplicitStepSimulator class for an example.
      *
      * @param begin   An iterator to the first body that should be evaluated.
      * @param end     An iterator to one past the last body that should be evaluated.
      */
      void evaluate(indirect_body_iterator begin,indirect_body_iterator end)
      {
        for(indirect_body_iterator body=begin;body!=end;++body)
          evaluate(&(*body));
      }

      void clear(){}

    };

  } // namespace mbd

} // namespace OpenTissue

#endif // OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_KINETIC_ENERGY_SLEEPY_POLICY_H
