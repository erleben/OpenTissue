#ifndef OPENTISSUE_KINEMATICS_SKINNING_GL_SKIN_RENDER_H
#define OPENTISSUE_KINEMATICS_SKINNING_GL_SKIN_RENDER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/graphics/core/gl/gl_util.h>
#include <OpenTissue/kinematics/skinning/skinning_gpu_vertex.h>
#include <OpenTissue/kinematics/skinning/sbs/skinning_create_key.h>

namespace OpenTissue
{

  namespace gl
  {

    class SkinRender
    {
    private:
      GLuint*   m_BufferID;
      size_t    m_bufferSz;
      size_t*   m_vertexCount;

    public:
      template<typename skin_type>
      void init( skin_type & skin,
                 typename skin_type::skin_part_type::no_gpu_support* p=0 ) // Use SFINAE
      {
      }

      template<typename skin_type>
      void init( skin_type & skin,
                 typename skin_type::skin_part_type::gpu_support* p=0 ) // Use SFINAE
      {
        createGPUBuffers( skin );
      }

      template <typename skin_type,typename skeleton_type>
      void render( skin_type & skin, 
                   skeleton_type & skeleton,
                   typename skin_type::skin_part_type::no_gpu_support* p=0 ) // Use SFINAE
      {
        // Do prerendering
        skin_type::skin_part_type::pre_render( skin );

        // Update the skin = update skin mesh according to skeleton
        skin.update( skeleton );

        // Just use OpenGl immediate mode to draw skin
        for(unsigned int i=0;i<skin.m_sz;++i)
        {
          if(skin.m_skin_parts[i].m_material_idx!=-1)
            skin.m_material[skin.m_skin_parts[i].m_material_idx].use();

          gl::DrawMesh( skin.m_skin_parts[i] );
        }

        // Do postrendering
        skin_type::skin_part_type::post_render( skin );
      }

      // TODO: remake const and remove skin. ... .func(skin)
      template <typename skin_type,typename skeleton_type>
      void render( skin_type & skin, 
                   skeleton_type & skeleton,
                   typename skin_type::skin_part_type::gpu_support* p=0 ) // Use SFINAE
      {
        // Do prerendering, enable Cg etc.
        skin_type::skin_part_type::pre_render( skin );

        // Update the skeleton = convert it to GPU form
        skin.update( skeleton );

        unsigned int stride = sizeof(OpenTissue::skinning::gpu_vertex);

        GLvoid * ptr_vert   = BUFFER_OFFSET(0);
        GLvoid * ptr_norm   = BUFFER_OFFSET(4*sizeof(float));
        GLvoid * ptr_weig   = BUFFER_OFFSET(7*sizeof(float));
        GLvoid * ptr_bidx   = BUFFER_OFFSET(11*sizeof(float));


        // Enable Vertex Arrays, and bind buffer
        glEnableClientState( GL_VERTEX_ARRAY );
        glEnableClientState( GL_NORMAL_ARRAY );
        glEnableClientState( GL_COLOR_ARRAY );
        glEnableClientState( GL_TEXTURE_COORD_ARRAY );

        for( size_t i=0; i<m_bufferSz; ++i )
        {
          if(skin.m_skin_parts[i].m_material_idx!=-1)
            skin.m_material[skin.m_skin_parts[i].m_material_idx].use();

          glBindBuffer( GL_ARRAY_BUFFER, m_BufferID[i] );

          // Setup pointers
          glVertexPointer  ( 4, GL_FLOAT, stride, ptr_vert );
          glNormalPointer  (    GL_FLOAT, stride, ptr_norm );
          glColorPointer   ( 4, GL_FLOAT, stride, ptr_weig );
          glTexCoordPointer( 4, GL_FLOAT, stride, ptr_bidx );

          // Draw vertex arrays
          glDrawArrays( GL_TRIANGLES, 0, (GLsizei)m_vertexCount[i] );
        }

        // Disable vertex arrays again
        glDisableClientState( GL_VERTEX_ARRAY );
        glDisableClientState( GL_NORMAL_ARRAY );
        glDisableClientState( GL_COLOR_ARRAY );
        glDisableClientState( GL_TEXTURE_COORD_ARRAY );

        // Do postrendering, enable Cg etc.
        skin_type::skin_part_type::post_render( skin );
      }

      void cleanup()
      {
        glDeleteBuffers( (GLsizei)m_bufferSz, m_BufferID );
        delete[] m_BufferID;
      }

    protected:
      template<typename skin_type>
      void createGPUBuffers( skin_type & skin )
      {
        typedef typename skin_type::skin_part_type     skin_part_type;

        skin_part_type* meshArr = skin.m_skin_parts;

        // Allocate id's for GPU buffers
        m_bufferSz = skin.m_sz;
        m_BufferID = new GLuint[m_bufferSz];
        m_vertexCount = new size_t[m_bufferSz];
        for(size_t i=0; i<m_bufferSz; ++i)
        {
          m_BufferID[i] = 0; // NULL;
          m_vertexCount[i] = 0; // Initialize
        }

        // Generate GPU buffers
        glGenBuffers( (GLsizei)m_bufferSz, m_BufferID );

        // First get size
        for(size_t i=0; i<m_bufferSz; ++i)
          m_vertexCount[i] = 3*meshArr[i].size_faces();

        OpenTissue::skinning::gpu_vertex* vertexData = NULL;

        // Now compute all vertices
        for(size_t i=0; i<m_bufferSz; ++i)
        {
          // Allocate gpu vertex array
          vertexData = new OpenTissue::skinning::gpu_vertex[m_vertexCount[i]];

          // Make pointer copy - this will be incremented during creation!
          OpenTissue::skinning::gpu_vertex* data = vertexData;

          typename skin_part_type::const_face_iterator end = meshArr[i].face_end();
          typename skin_part_type::const_face_iterator f   = meshArr[i].face_begin();

          for(;f!=end;++f)
          {
            typename skin_part_type::const_face_vertex_circulator v( *f );

            for( int j=0; j<3; ++j )
            {
              data->vertex[0] = v->m_coord(0);
              data->vertex[1] = v->m_coord(1);
              data->vertex[2] = v->m_coord(2);

              // Get rc index from the lut
              data->vertex[3] = skin.m_rc_lut[v->m_key]; 

              data->normal[0] = v->m_normal(0);
              data->normal[1] = v->m_normal(1);
              data->normal[2] = v->m_normal(2);

              data->weight[0] = v->m_weight[0];
              data->weight[1] = v->m_weight[1];
              data->weight[2] = v->m_weight[2];
              data->weight[3] = v->m_weight[3];

              data->boneIdx[0] = v->m_bone[0];
              data->boneIdx[1] = v->m_bone[1];
              data->boneIdx[2] = v->m_bone[2];
              data->boneIdx[3] = v->m_bone[3];

              ++v; 
              ++data;
            }
          }

          // Select the current GPU buffer
          glBindBuffer( GL_ARRAY_BUFFER, m_BufferID[i] );

          // Upload skin part to GPU buffer
          glBufferData( GL_ARRAY_BUFFER, m_vertexCount[i]*sizeof(OpenTissue::skinning::gpu_vertex), vertexData, GL_STATIC_DRAW );

          delete[] vertexData;
        }
      }
    };

  }

} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_GL_SKIN_RENDER_H
#endif
