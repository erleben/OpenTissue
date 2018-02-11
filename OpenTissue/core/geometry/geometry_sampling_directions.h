#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SAMPLING_DIRECTIONS_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SAMPLING_DIRECTIONS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_make_sphere.h> 
#include <OpenTissue/core/containers/mesh/common/util/mesh_make_sphere.h> 

#include <boost/cast.hpp> //--- Needed for boost::numeric_cast
#include <cmath>
#include <vector>

namespace OpenTissue
{
  namespace geometry
  {


    namespace detail
    {

      /**
      * Auxiliary class used to pick the sampling strategy type.
      */
      class SamplingTypePicker
      {
      public:

        typedef enum { sphere_type, random_type, tetrahedron_type, icosahedron_type } sampling_type;

      protected:

        static sampling_type & type() 
        {   
          static sampling_type tmp = icosahedron_type;
          return tmp;
        }

      public:

        static sampling_type get_type() 
        {   
          changed() = false;
          return type();
        }

        static void set_type(sampling_type new_type) 
        {   
          changed() = true;
          type() = new_type;
        }

        static bool & changed() 
        {
          static bool tmp = true;
          return tmp; 
        }
      };

      /**
      * Auxiliary class used to setup and retrieve sampling directions.
      */
      template<typename vector3_type>
      class SamplingDirections
      {
      public:

        typedef enum { sphere_type, random_type, tetrahedron_type, icosahedron_type} sampling_type;

      protected:

        typedef polymesh::PolyMesh<>  mesh_type;

      protected:

        static std::vector<vector3_type> & dir()
        {
          static std::vector<vector3_type> m_dir;   ///< Sampling directions
          return m_dir;
        }

      public:

        SamplingDirections()
        {
          init();
        }

      public:

        void init()
        {
          using std::sqrt;

          typedef typename vector3_type::value_type  real_type;

          bool changed = SamplingTypePicker::changed();
          SamplingTypePicker::sampling_type  type = SamplingTypePicker::get_type();

          if(!changed && !dir().empty() )
            return;

          unsigned int max_samples = 0;

          switch( type )
          {
          case SamplingTypePicker::icosahedron_type:
            {
              std::cout << "SamplingDirections::init(): Using icosahedron type" << std::endl;
              mesh_type sphere;
              make_sphere(1.0,3,sphere);
              max_samples = static_cast<unsigned int>( sphere.size_vertices() ); 
              dir().resize(max_samples);
              typename mesh_type::vertex_iterator v = sphere.vertex_begin();
              for(unsigned int k=0;k<max_samples;++k,++v)
                dir()[k] = unit(v->m_coord);
            }
            break;
          case SamplingTypePicker::tetrahedron_type:
            {
              std::cout << "SamplingDirections::init(): Using tetrahedron type" << std::endl;
              //--- Seems to merge curves together? Density of points is not as even as I would like?
              //--- construct tetrahedron with all edges having same length
              mesh_type sphere;
              make_sphere(1.0,4,sphere,true);
              max_samples = static_cast<unsigned int>( sphere.size_vertices() ); 
              dir().resize(max_samples);
              typename mesh_type::vertex_iterator v = sphere.vertex_begin();
              for(unsigned int k=0;k<max_samples;++k,++v)
                dir()[k] = unit(v->m_coord);
            }
            break;
          case SamplingTypePicker::random_type:
            {
              std::cout << "SamplingDirections::init(): Using random type" << std::endl;
              //--- seens to create curly curves...
              max_samples = 642; 
              dir().resize(max_samples);
              for(unsigned int k=0;k<max_samples;++k)
              {
                random(dir()[k],-1,1);
                dir()[k] = unit(dir()[k]);
              }
            }
            break;
          case SamplingTypePicker::sphere_type:
            {
              std::cout << "SamplingDirections::init(): Using sphere type" << std::endl;
              //--- this actually looks the best eventhough there is higher density at the poles
              mesh_type sphere;
              mesh::make_sphere(1.0,25,25,sphere);
              max_samples = static_cast<unsigned int>( sphere.size_vertices() ); 
              dir().resize(max_samples);
              typename mesh_type::vertex_iterator v = sphere.vertex_begin();
              for(unsigned int k=0;k<max_samples;++k,++v)
                dir()[k] = unit(v->m_coord);
            }
            break;
          };
          std::cout << "sample_directions::init(): sampling directions = " << max_samples << std::endl;
        }

        std::size_t size() const {  return dir().size(); }

        vector3_type const & operator()(unsigned int idx) const {   return dir()[idx];  }

      };

    } //namespace detail

  } // namespace geometry
} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SAMPLING_DIRECTIONS_H
#endif
