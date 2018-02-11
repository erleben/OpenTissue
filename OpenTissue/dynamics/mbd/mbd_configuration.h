#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_CONFIGURATION_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_CONFIGURATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/containers_hash_map.h>
#include <OpenTissue/utility/utility_map_data_iterator.h>

#include <boost/iterator/indirect_iterator.hpp>

namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types >
    class Configuration
    {
    public:

      typedef typename mbd_types::body_type                      body_type;
      typedef typename mbd_types::edge_type                      edge_type;
      typedef typename mbd_types::material_library_type          material_library_type;
      typedef typename mbd_types::math_policy::index_type        size_type;

    protected:

      typedef typename mbd_types::math_policy::real_type         real_type;
      typedef typename mbd_types::joint_type                     joint_type;
      typedef typename mbd_types::group_type                     group_type;
      typedef typename mbd_types::collision_detection_policy     collision_detection_policy;


      typedef typename stdext::hash_map<size_type, body_type*>   body_ptr_lut_type;
      typedef typename stdext::hash_map<size_type, joint_type*>  joint_ptr_lut_type;
      typedef typename stdext::hash_map<size_type, edge_type>    edge_lut_type;

    protected:

      collision_detection_policy * m_collision_detection;  ///< Collision Detection Engine.
      material_library_type    *   m_material_library;     ///< material_type library
      group_type                   m_all;                  ///< Internal data strucure used by the method ``get_all_body_group()''
      body_ptr_lut_type            m_bodies;               ///< All bodies currently added to the simulator.
      joint_ptr_lut_type           m_joints;               ///< All joints currently added to the simulator.
      edge_lut_type                m_edges;                ///< All edges that currently exist in the simulator.
      real_type                    m_collision_envelope;   ///< Size of collision envelope

    protected:

      typedef OpenTissue::utility::map_data_iterator<typename body_ptr_lut_type::iterator>       body_ptr_lut_iterator;
      typedef OpenTissue::utility::map_data_iterator<typename joint_ptr_lut_type::iterator>      joint_ptr_lut_iterator;

    public:

      typedef boost::indirect_iterator<body_ptr_lut_iterator>               body_iterator;
      typedef boost::indirect_iterator<joint_ptr_lut_iterator>              joint_iterator;
      typedef OpenTissue::utility::map_data_iterator<typename edge_lut_type::iterator>           edge_iterator;

      body_iterator body_begin() { return body_iterator( body_ptr_lut_iterator(m_bodies.begin()) );}
      body_iterator body_end()   { return body_iterator( body_ptr_lut_iterator(m_bodies.end()  ) );}

      edge_iterator edge_begin() { return edge_iterator( m_edges.begin() );}
      edge_iterator edge_end()   { return edge_iterator( m_edges.end()   );}

      joint_iterator joint_begin() { return joint_iterator( joint_ptr_lut_iterator(m_joints.begin()) );}
      joint_iterator joint_end()   { return joint_iterator( joint_ptr_lut_iterator(m_joints.end()  ) );}

      size_type size_bodies() const {return m_bodies.size(); }

    public:

      Configuration()
        : m_collision_detection(0)
        , m_material_library(0)
        , m_collision_envelope(0.01)
      {}

      ~Configuration()
      { 
        clear();
      }

    public:

      void set_material_library(material_library_type & library) {  this->m_material_library = &library; }

      material_library_type * get_material_library() const { return this->m_material_library; }

      void set_collision_envelope(real_type const & value)
      {  
        assert(value>0 || !"Configuration::set_collision_envelope(): envelope must be positive");
        m_collision_envelope = value; 
      }

      real_type const & get_collision_envelope() const { return m_collision_envelope; }

      /**
      * Collision Detection Connect Method.
      * Used by simulator class to connect configuration with collision
      * detection engine. 
      *
      * @param collision_detection           The Collision Detection Engine.
      */
      void connect(collision_detection_policy & collision_detection)
      {
        if(m_collision_detection)
          m_collision_detection->clear();

        m_collision_detection = &collision_detection;

        for(body_iterator body = body_begin();body!=body_end();++body)
        {
          m_collision_detection->add(&(*body));
        }
      }

    public:

      /**
      * Get All body_type group_type.
      * This method sets us a body group, containing (ONLY) all currently
      * active bodies in the configuration at the time of invocation.
      *
      * @return       A pointer to a BodyGroup, containting all active bodies.
      */
      group_type * get_all_body_group()
      {
        m_all.clear();
        for(body_iterator body = body_begin();body!=body_end();++body)
        {
          if(body->is_active())
            m_all.m_bodies.push_back(&(*body));
        }
        return &m_all;
      }

      bool add(body_type * body)
      {
        assert(body || !"Configuration::add(): body was null");
        assert(m_bodies.find(body->get_index())==m_bodies.end() || !"Configuration::add(): body was already in configuration");

        m_bodies.insert( std::pair< size_type, body_type* >( body->get_index(), body ) );

        if(m_collision_detection)
          m_collision_detection->add(body);

        return true;
      }

      bool remove(body_type * body)
      {
        assert(body || !"Configuration::remove(): body was null");
        assert(m_bodies.find(body->get_index())!=m_bodies.end() || !"Configuration::remove(): body was not in configuration");

        if(body->has_joints())
        {
          std::cout << "Configuration::remove(): Remove all joints on body first" << std::endl;
          return false;
        }

        while(!body->m_edges.empty())
        {
          edge_type * edge = body->m_edges.front();
          edge->get_body_A()->m_edges.remove(edge);
          edge->get_body_B()->m_edges.remove(edge);
          m_edges.erase(edge->hash_key());
        }

        m_bodies.erase(body->get_index());

        if(m_collision_detection)
          m_collision_detection->remove(body);
        return true;
      }

      bool add(joint_type * joint)
      {
        assert(joint || !"Configuration::add(): Joint was null");
        assert(m_joints.find(joint->get_index())==m_joints.end() || !"Configuration::add(): Joint was already in configuration");
        if(joint->get_socket_A()==0)
        {
          std::cout << "Configuration::add(): Joint were missing socket A" << std::endl;
          return false;
        }
        if(joint->get_socket_A()->get_body()==0)
        {
          std::cout << "Configuration::add(): Joint socket A were missing a body" << std::endl;
          return false;
        }
        if(joint->get_socket_B()==0)
        {
          std::cout << "Configuration::add(): Joint were missing socket B" << std::endl;
          return false;
        }
        if(joint->get_socket_B()->get_body()==0)
        {
          std::cout << "Configuration::add(): Joint socket B were missing a body" << std::endl;
          return false;
        }
        typename body_ptr_lut_type::iterator itA = m_bodies.find(joint->get_socket_A()->get_body()->get_index());
        if(itA==m_bodies.end())
        {
          std::cout << "Configuration::add(): First add body on socket A" << std::endl;
          return false;
        }
        typename body_ptr_lut_type::iterator itB = m_bodies.find(joint->get_socket_B()->get_body()->get_index());
        if(itB==m_bodies.end())
        {
          std::cout << "Configuration::add(): First add body on socket B" << std::endl;
          return false;
        }
        m_joints.insert( std::pair< size_type, joint_type* >( joint->get_index(), joint ) );
        return true;
      }

      bool remove(joint_type * joint)
      {
        assert(joint || !"Configuration::remove(): Joint was null");
        assert(m_joints.find(joint->get_index())!=m_joints.end() || !"Configuration::remove(): Joint was not in configuration");      
        assert( !(joint->get_body_A()) || !"Configuration::remove(): Body A on joint was non-null");
        assert( !(joint->get_body_B()) || !"Configuration::remove(): Body B on joint was non-null");

        m_joints.erase(joint->get_index());

        return true;
      }

      edge_type * get_edge(body_type * A,body_type * B)
      {
        assert(A    || !"Configuration::get_edge(): body A was null");
        assert(B    || !"Configuration::get_edge(): body B was null");
        assert(A!=B || !"Configuration::get_edge(): body A and B were the same");

        typename edge_lut_type::iterator edge = m_edges.find(edge_type::hash_key(A,B));
        if(edge==m_edges.end())
          return 0;
        return &(edge->second);
      }

      /**
      * Add edge_type.
      *
      * @param A     A pointer to one of the incident bodies of the edge.
      * @param B     A pointer to the other incident body of the edge.
      *
      * @return      A pointer to the newly added edge.
      */
      edge_type * add(body_type * A,body_type * B)
      {
        assert(A    || !"Configuration::add(): body A was null");
        assert(B    || !"Configuration::add(): body B was null");
        assert(A!=B || !"Configuration::add(): body A and B were the same");
        assert(m_material_library || !"Configuration::add(): Material library was null");
        assert(m_edges.find(edge_type::hash_key(A,B))==m_edges.end() || !"Configuration::add(): Edge allready existed in configuration");
        assert(m_collision_detection || !"Configuration::add(): Collision Detection was null");
      
        std::pair<typename edge_lut_type::iterator,bool> result = m_edges.insert(  std::make_pair(edge_type::hash_key(A,B),edge_type())   );
        
        edge_type * edge = &(result.first->second);
        edge->init(A,B);
        edge->get_body_A()->m_edges.push_back(edge);
        edge->get_body_B()->m_edges.push_back(edge);

        edge->m_material = m_material_library->get(A->get_material_idx(),B->get_material_idx());

        if(!edge->m_material)
        {
          std::cout << "Configuration::add(): Could not find a material between ("<< A->get_material_idx() <<","<< B->get_material_idx() <<") using default material" << std::endl;
          edge->m_material = m_material_library->default_material();
        }

        edge->m_collision_detection = m_collision_detection;

        return edge;
      }

      void remove(edge_type * edge)
      {
        assert(edge || !"Configuration::remove(): edge was null");
        assert(m_edges.find( edge->hash_key() )!=m_edges.end() || !"Configuration::remove(): edge was not in configuration");

        edge->get_body_A()->m_edges.remove(edge);
        edge->get_body_B()->m_edges.remove(edge);
        m_edges.erase(edge->hash_key());
      }

      void clear()
      {
        m_collision_detection = 0;
        m_material_library    = 0;

        while(!m_joints.empty())
        {
          typename joint_ptr_lut_type::iterator it = m_joints.begin();
          joint_type * joint = it->second;
          joint->clear();
          remove(joint);
        }

        while(!m_bodies.empty())
        {
          typename body_ptr_lut_type::iterator it = m_bodies.begin();
          body_type * body = it->second;
          remove(body);
          body->clear();
        }

        assert(m_bodies.empty() || !"Configuration::clear(): Internal error, not all bodies where removed?");
        assert(m_joints.empty() || !"Configuration::clear(): Internal error, not all joints where removed?");

        // Note that edges has been removed implicitly. This happens
        // while the bodies are removed. See method remove(body)
        assert(m_edges.empty() || !"Configuration::clear(): Internal error, not all edges where removed?");
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_CONFIGURATION_H
#endif
