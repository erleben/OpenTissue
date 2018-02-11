#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_SIMULATOR_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_SIMULATOR_INTERFACE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_scripted_motions.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Simulation Time Handler Class.
    * All simulators should be inherited from this class.
    * Also the run-methods in each simulator implementation are responsible
    * for updating the simulation time.
    */
    template <typename mbd_types>
    class SimulatorInterface
    {
    public:

      typedef mbd_types  types;

    protected:

      typedef typename mbd_types::math_policy         math_policy;
      typedef typename math_policy::real_type         real_type;
      typedef typename math_policy::value_traits      value_traits;

      typedef typename mbd_types::configuration_type             configuration_type;
      typedef typename mbd_types::sleepy_policy                  sleepy_policy;
      typedef typename mbd_types::stepper_policy                 stepper_policy;
      typedef typename mbd_types::collision_detection_policy     collision_detection;

    private:

      real_type                  m_time;                 ///< The current simulation time.
      collision_detection        m_collision_detection;  ///< Collision Detection Engine.
      configuration_type *       m_configuration;        ///< configuration_type, contains all the stuff that should be simulated.
      sleepy_policy              m_sleepy;               ///< Sleepy strategy, examines bodies and determines which ones to put to sleep.
      stepper_policy             m_stepper;              ///< Stepper routine (takes care of the actual simulation)

    public:

      stepper_policy       * get_stepper()       { return &m_stepper; } 
      stepper_policy const * get_stepper() const { return &m_stepper; } 
      sleepy_policy        * get_sleepy()        { return &m_sleepy;  } 
      sleepy_policy  const * get_sleepy()  const { return &m_sleepy;  } 

      collision_detection const * get_collision_detection() const 
      { 
        assert(m_collision_detection || !"SimulatorInterface::get_collision_detection(): null pointer, did you forget to invoke init?");
        return m_collision_detection; 
      } 

      configuration_type  const * get_configuration()       const 
      { 
        assert(m_configuration || !"SimulatorInterface::get_configuration(): null pointer, did you forget to invoke init?");
        return m_configuration; 
      } 

      collision_detection  * get_collision_detection()        {         return &m_collision_detection;       }

      configuration_type   * get_configuration()        
      { 
        assert(m_configuration || !"SimulatorInterface::get_configuration(): null pointer, did you forget to invoke init?");
        return m_configuration;       
      } 

    public:

      SimulatorInterface( )
        : m_time( value_traits::zero() )
        , m_configuration(0)
      {}

      virtual ~SimulatorInterface(){}

    public:

      virtual  void run(real_type const & time_step) = 0;

      void init(configuration_type & configuration)
      {
        this->reset_time();
        this->m_configuration = &configuration;
        m_collision_detection.clear();
        m_collision_detection.init(configuration);
        configuration.connect(m_collision_detection);
        m_stepper.connect(configuration);
        mbd::compute_scripted_motions(*(this->m_configuration->get_all_body_group()),this->time());
      }

      void clear()
      {
        this->reset_time();
        this->m_configuration = 0;
        m_collision_detection.clear();
        m_stepper.clear();
        m_sleepy.clear();
      }

    public:

      real_type const & time( ) const { return m_time; }

      void reset_time() { m_time = value_traits::zero(); }

    protected:

      void update_time( real_type const & time_step )
      {
        assert( time_step >= value_traits::zero() || !"SimulatorInterface::update_time(): time step value must be non-negative");
        m_time += time_step;
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_SIMULATOR_INTERFACE_H
#endif
