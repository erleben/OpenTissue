#ifndef OPENTISSUE_DYNAMICS_EDM_EDM_SYSTEM_H
#define OPENTISSUE_DYNAMICS_EDM_EDM_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <map>
#include <cassert>

namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types_>
    class System
    {
    public:
      typedef edm_types_                          edm_types;
      typedef typename edm_types::value_traits    value_traits;
      typedef typename edm_types::model_type      model_type;
      typedef typename edm_types::force_type      force_type;
      typedef typename edm_types::object_type     object_type;

      typedef std::map<std::string, force_type *>       EDMIOForces;
      typedef std::map<std::string, object_type *>      EDMIOObjects;
      typedef std::map<std::string, model_type *>       EDMIOModels;

    public:

      System()
      {}

      virtual ~System()
      {
        reset();
      }

    public:

      void run(bool compute_elasticity = false)
      {
        typename EDMIOModels::iterator model, end = m_models.end();
        for (model = m_models.begin(); model != end; ++model)
          model->second->run(compute_elasticity);
      }

    public:

      void reset()
      {
        //--- KE 2006-05-24: Used boost::shared_ptr instead!

        // free memory for all deformable bodies
        for (typename EDMIOModels::iterator iob = m_models.begin(); iob != m_models.end(); ++iob)
          delete iob->second;
        // free all deformable body containers
        m_models.clear();

        // free memory for all objects
        for (typename EDMIOObjects::iterator ioo = m_objects.begin(); ioo != m_objects.end(); ++ioo)
          delete ioo->second;
        // free all object containers
        m_objects.clear();

        // free memory for all forces
        for (typename EDMIOForces::iterator iof = m_forces.begin(); iof != m_forces.end(); ++iof)
          delete iof->second;
        // free all force containers
        m_forces.clear();
      }

      template<typename edm_force>
      edm_force * create_force(std::string const & id)
      {
        if (m_forces.end() != m_forces.find(id))
          return 0;
        edm_force* f = new edm_force;
        m_forces[id] = f;
        return f;
      }

      void delete_force(std::string const & id)
      {
        typename EDMIOForces::iterator f = m_forces.find(id);
        if (m_forces.end() == f)
          return 0;
        delete f->second;
        m_forces.erase(f);
      }

      force_type const * get_force(std::string const & id) const
      {
        typename EDMIOForces::const_iterator f = m_forces.find(id);
        if (m_forces.end() == f)
          return 0;
        return f->second;
      }

      template<typename shape_type>
      shape_type * create_object(std::string const & id)
      {
        if (m_objects.end() != m_objects.find(id))
          return 0;
        object_type * o = new object_type;
        m_objects[id] = o;
        return &o->template create_shape<shape_type>();
      }

      void delete_object(std::string const & id)
      {
        typename EDMIOObjects::iterator o = m_objects.find(id);
        if (m_objects.end() == o)
          return 0;
        delete o->second;
        m_objects.erase(o);
      }

      object_type const * get_object(std::string const & id) const
      {
        typename EDMIOObjects::const_iterator o = m_objects.find(id);
        if (m_objects.end() == o)
          return 0;
        return o->second;
      }

      object_type * get_object(std::string const & id)
      {
        typename EDMIOObjects::const_iterator o = m_objects.find(id);
        if (m_objects.end() == o)
          return 0;
        return o->second;
      }

      template<typename edm_model>
      edm_model * create_model(std::string const & id)
      {
        if (m_models.end() != m_models.find(id))
          return 0;
        edm_model* m = new edm_model;
        m_models[id] = m;
        return m;
      }

      void delete_model(std::string const & id)
      {
        typename EDMIOModels::iterator b = m_models.find(id);
        if (m_models.end() == b)
          return 0;
        delete b->second;
        m_models.erase(b);
      }

      model_type const * get_model(std::string const & id) const
      {
        typename EDMIOModels::const_iterator b = m_models.find(id);
        if (m_models.end() == b)
          return 0;
        return b->second;
      }

      model_type & get_model(size_t idx)
      {
        typename EDMIOModels::iterator tmp = m_models.begin();
        while (idx--) ++tmp;
        assert(m_models.end() != tmp);
        return *tmp->second;
      }

      size_t model_count() const
      {
        return size_t(static_cast<size_t>(m_models.size()));
      }

      EDMIOForces const & forces() const
      {
        return m_forces;
      }

      EDMIOObjects const & objects() const
      {
        return m_objects;
      }

      EDMIOModels const & models() const
      {
        return m_models;
      }

    private:

      EDMIOForces    m_forces;
      EDMIOObjects   m_objects;
      EDMIOModels    m_models;

    };

  }  // namespace edm

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_EDM_SYSTEM_H
#endif
