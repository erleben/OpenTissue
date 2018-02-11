#ifndef OPENTISSUE_KINEMATICS_SKINNING_SKINNING_ANIMATED_SKIN_H
#define OPENTISSUE_KINEMATICS_SKINNING_SKINNING_ANIMATED_SKIN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/gl/gl_util.h>
#include <OpenTissue/core/containers/mesh/trimesh/trimesh.h>
#include <OpenTissue/core/containers/mesh/trimesh/util/trimesh_util.h>
#include <OpenTissue/kinematics/skinning/skinning_traits.h>
#include <OpenTissue/utility/gl/gl_draw_mesh.h>
#include <cassert>
#include <vector>

// Shader program header includes
#include <OpenTissue/gpu/cg/cg_program.h>

namespace OpenTissue
{
  namespace skinning
  {

    template<typename character_types>
    class AnimatedSkin
    {
    public:
      typedef typename character_types::skin_part_type              skin_part_type;
      typedef typename character_types::key_type                    key_type;
      typedef typename character_types::math_types::vector3_type	  vector3_type;

      // Comtainiers for rotation center lookup
      typedef std::vector<vector3_type>         rc_cached_type;
      typedef std::map<key_type, size_t>        lut_type;

      //--- Shader program
      typedef OpenTissue::cg::Program	            cg_program_type;

    public:
      static const size_t			    m_sz = 20u;			    ///< Number of skin parts.
      skin_part_type		          m_skin_parts[m_sz];	///< Storage of skins.
      OpenTissue::gl::Material    m_material[m_sz];   ///< Storage of skin materials.

      bool						            m_uploadBones;		  ///< Flag. Only upload bones to gpu on first skin.
      bool						            m_uploadRc;			    ///< Flag. Only upload r_c to gpu on first skin.
      size_t					            m_num_keys;			    ///< Numer of unique keys for this skin.

      cg_program_type		          m_vp;				        ///< Cg vertex program.

      rc_cached_type              m_rc_cached;		    ///< Cached rotation centers.
      lut_type                    m_rc_lut;			      ///< Lookup Table for rotation centers.

    public:
      AnimatedSkin()
        : m_uploadBones( true ),
        m_uploadRc( true ),
        m_num_keys( 0u )
      {
      }

      virtual ~AnimatedSkin()
      {
      }

      void init()
      {
        skin_part_type::init_skin_render( *this );

        // Create a unique key for each vertex
        // Actually only used for sbs gpu skinning
        unsigned int idx = 0;

        for(size_t i=0u; i<m_sz; ++i)
        {
          typename skin_part_type::vertex_iterator vertex = m_skin_parts[i].vertex_begin();
          typename skin_part_type::vertex_iterator end    = m_skin_parts[i].vertex_end();

          for(;vertex!=end;++vertex)
          {
            vertex->m_key = create_key( vertex );

            if( vertex->m_influences > 1 )
            {
              typename lut_type::iterator lut_it = m_rc_lut.find( vertex->m_key );
              if( lut_it == m_rc_lut.end() )
              {
                // Key not in lut...
                m_rc_lut[vertex->m_key] = idx;
                ++idx;
              }
            }
          }
        }

        // Tells sbs_gpu how many keys have been created
        m_num_keys = m_rc_lut.size();
      }

      template <typename skeleton_type>
      void update( skeleton_type & skeleton )
      {
        for(size_t i=0u; i<m_sz; ++i)
        {
          m_skin_parts[i].update( skeleton, *this );
        }
      }

      // Create a software_update and a gpu_update function, use SFINAE to distinguish 

      void cleanup()
      {
        skin_part_type::cleanup_skin_render( *this );
      }
    };

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_SKINNING_ANIMATED_SKIN_H
#endif
