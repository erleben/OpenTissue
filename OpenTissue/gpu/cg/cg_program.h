#ifndef OPENTISSUE_GPU_CG_CG_PROGRAM_H
#define OPENTISSUE_GPU_CG_CG_PROGRAM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/gpu/cg/cg_util.h>

#include<string>
#include<map>
#include<cassert>

namespace OpenTissue
{
  namespace cg
  {

    class Program
    {
    public:

      typedef enum  {vertex_program, fragment_program}  program_type;
      typedef std::map<std::string, std::string>        name_type_map;

    protected:

      program_type   m_type;             ///<
      CGprogram      m_program;          ///<
      CGprofile      m_profile;          ///<
      CGcontext      m_context;          ///<
      name_type_map  m_uniforms;         ///<
      bool           m_loaded;

    public:

      Program()
        : m_loaded(false) 
      {}

    public:

      void enable()
      {
        //switch(m_type)
        //{
        //case vertex_program:
        //  {
        //    if(OpenTissue::cg::vertex_program())
        //      OpenTissue::cg::vertex_program()->disable();
        //    OpenTissue::cg::vertex_program() = this;
        //  }
        //  break;
        //case fragment_program:
        //  {
        //    if(OpenTissue::cg::fragment_program())
        //      OpenTissue::cg::fragment_program()->disable();
        //    OpenTissue::cg::fragment_program() = this;
        //  }
        //  break;
        //};            
        cgGLEnableProfile(m_profile);
        cgGLBindProgram(m_program);
        gl::gl_check_errors("Program::enable() - at the end");
      }

      void disable()
      {
        cgGLUnbindProgram( m_profile );
        cgGLDisableProfile( m_profile );
        gl::gl_check_errors("Program::disable()");

        glDisable(GL_TEXTURE_2D);//--- KE 2005.10.23: Hmm, I thought cgGLUnbindProgram took care of this!!!
        glDisable(GL_TEXTURE_3D);//--- KE 2005.10.23: Hmm, I thought cgGLUnbindProgram took care of this!!!
      }

    public:

      bool load_from_text(program_type type,std::string const & program_text)
      {
        m_context = OpenTissue::cg::context();
        if(type==vertex_program)
        {
          m_profile = OpenTissue::cg::vertex_profile();
          if(OpenTissue::cg::vertex_program())
            OpenTissue::cg::vertex_program()->m_loaded = false;
        }
        else
        {
          m_profile = OpenTissue::cg::fragment_profile();
          if(OpenTissue::cg::fragment_program())
            OpenTissue::cg::fragment_program()->m_loaded = false;
        }
        const char *cstr = program_text.c_str();
        m_program = cgCreateProgram(m_context, CG_SOURCE, cstr, m_profile, 0, 0);

        if (m_program == 0)
        {
          std::cerr << "Program::load_from_text(): could not create program" << std::endl;
          return false;
        }
        cgGLLoadProgram( m_program );
        m_uniforms = get_uniforms_map();
        m_loaded = true;
        gl::gl_check_errors("After Program::load_from_text()");
        return true;
      };

      bool load_from_file(program_type type,std::string const & filename)
      {
        m_context = OpenTissue::cg::context();

        if(type==vertex_program)
        {
          m_profile = OpenTissue::cg::vertex_profile();
          if(OpenTissue::cg::vertex_program())
            OpenTissue::cg::vertex_program()->m_loaded = false;
        }
        else
        {
          m_profile = OpenTissue::cg::fragment_profile();
          if(OpenTissue::cg::fragment_program())
            OpenTissue::cg::fragment_program()->m_loaded = false;
        }
        const char *cstr = filename.c_str();
        m_program = cgCreateProgramFromFile(m_context, CG_SOURCE, cstr, m_profile, 0, 0);
        if (m_program == 0)
        {
          std::cerr << "Program::load_from_file("<<filename<<"): could not create program" << std::endl;
          return false;
        }
        cgGLLoadProgram( m_program );
        m_uniforms = get_uniforms_map();
        m_loaded = true;
        gl::gl_check_errors("After Program::load_from_file()");
        return true;
      };

      bool is_program_loaded()const
      {
        if(m_context != OpenTissue::cg::context())
          return false;
        return  m_loaded;
      }

      CGprogram get_program_ID(){  return m_program; }

    protected:

      struct RecurseParams
      {
        void operator()( name_type_map & map, CGparameter param )
        {
          if (!param)
            return;

          do
          {
            switch ( cgGetParameterType(param) )
            {
            case CG_STRUCT:
              RecurseParams()( map, cgGetFirstStructParameter( param ) );
              break;
            case CG_ARRAY:
              {
                int size = cgGetArraySize( param, 0 );
                for ( int i = 0; i < size; ++i )
                  RecurseParams()( map, cgGetArrayParameter( param, i ) );
              }
              break;
            case CG_BOOL:case CG_BOOL1:case CG_BOOL1x1: case CG_BOOL1x2:case CG_BOOL1x3:case CG_BOOL1x4:
            case CG_BOOL2:case CG_BOOL2x1: case CG_BOOL2x2:case CG_BOOL2x3:case CG_BOOL2x4:
            case CG_BOOL3:case CG_BOOL3x1: case CG_BOOL3x2:case CG_BOOL3x3:case CG_BOOL3x4:
            case CG_BOOL4:case CG_BOOL4x1: case CG_BOOL4x2:case CG_BOOL4x3:case CG_BOOL4x4:
            case CG_FIXED:case CG_FIXED1:case CG_FIXED1x1: case CG_FIXED1x2:case CG_FIXED1x3:case CG_FIXED1x4:
            case CG_FIXED2:case CG_FIXED2x1: case CG_FIXED2x2:case CG_FIXED2x3:case CG_FIXED2x4:
            case CG_FIXED3:case CG_FIXED3x1: case CG_FIXED3x2:case CG_FIXED3x3:case CG_FIXED3x4:
            case CG_FIXED4:case CG_FIXED4x1: case CG_FIXED4x2:case CG_FIXED4x3:case CG_FIXED4x4:
            case CG_FLOAT:case CG_FLOAT1:case CG_FLOAT1x1: case CG_FLOAT1x2:case CG_FLOAT1x3:case CG_FLOAT1x4:
            case CG_FLOAT2:case CG_FLOAT2x1: case CG_FLOAT2x2:case CG_FLOAT2x3:case CG_FLOAT2x4:
            case CG_FLOAT3:case CG_FLOAT3x1: case CG_FLOAT3x2:case CG_FLOAT3x3:case CG_FLOAT3x4:
            case CG_FLOAT4:case CG_FLOAT4x1: case CG_FLOAT4x2:case CG_FLOAT4x3:case CG_FLOAT4x4:
            case CG_HALF:case CG_HALF1:case CG_HALF1x1: case CG_HALF1x2:case CG_HALF1x3:case CG_HALF1x4:
            case CG_HALF2:case CG_HALF2x1: case CG_HALF2x2:case CG_HALF2x3:case CG_HALF2x4:
            case CG_HALF3:case CG_HALF3x1: case CG_HALF3x2:case CG_HALF3x3:case CG_HALF3x4:
            case CG_HALF4:case CG_HALF4x1: case CG_HALF4x2:case CG_HALF4x3:case CG_HALF4x4:
            case CG_INT:case CG_INT1:case CG_INT1x1: case CG_INT1x2:case CG_INT1x3:case CG_INT1x4:
            case CG_INT2:case CG_INT2x1: case CG_INT2x2:case CG_INT2x3:case CG_INT2x4:
            case CG_INT3:case CG_INT3x1: case CG_INT3x2:case CG_INT3x3:case CG_INT3x4:
            case CG_INT4:case CG_INT4x1: case CG_INT4x2:case CG_INT4x3:case CG_INT4x4:
            case CG_SAMPLERRECT:case CG_SAMPLER1D:case CG_SAMPLER2D:case CG_SAMPLER3D:
            case CG_SAMPLERCUBE:case CG_TYPE_START_ENUM:case CG_UNKNOWN_TYPE:
            case CG_TEXTURE:case CG_PROGRAM_TYPE:case CG_STRING:
              //// 2007-10-03 kenny: Added enumerators that were not explicitly handled by a case label
              //case CG_SAMPLER:
              //case CG_PIXELSHADER_TYPE:
              //case CG_VERTEXSHADER_TYPE:
              //case CG_SAMPLERCUBEARRAY:
              //case CG_SAMPLER1DARRAY:
              //case CG_SAMPLER2DARRAY:

            default:
              if ( cgGetParameterVariability( param ) == CG_UNIFORM )
              {
                //std::cout << "Program()::RecurseParams: found " << cgGetParameterName(param) << std::endl;
                map[cgGetParameterName(param)] = cgGetTypeString(cgGetParameterType(param));

              }
              break;
            }
          }
          while((param = cgGetNextParameter(param)) != 0);
        }
      };

    public:

      name_type_map get_uniforms_map()
      {
        name_type_map map;
        CGparameter cgp = cgGetFirstParameter( m_program, CG_PROGRAM );
        RecurseParams()( map, cgp );
        return map;
      }

    protected:

#ifndef NDEBUG
      template<typename texture_type>
      CGparameter get_texture_param( std::string const & name , texture_type const & texture) const
      {
        CGparameter param = cgGetNamedParameter( m_program, name.c_str() );
        if ( param == 0 )
        {
          std::cerr << "Program::get_texture_param(): Variable " << name.c_str() << " was not found or illegal." << std::endl;
          return 0;
        }
        name_type_map::const_iterator lookup = m_uniforms.find( name );
        assert( lookup != m_uniforms.end() || !"Program::get_texture_param(): The uniform map does not correspond with the Cg runtime." );
        std::string sampler_type(cgGetTypeString(cgGetParameterType(param)));
        if(!texture.is_valid_sampler_type(sampler_type))
        {
          std::cerr << "Program::get_texture_param(): Texture variable " << name.c_str() << " was not compatible with texture." << std::endl;
          return 0;
        };
        return param;
      }
#else
      template<typename texture_type>
      CGparameter get_texture_param( std::string const & name , texture_type const & /*texture*/) const
      {
        return cgGetNamedParameter( m_program, name.c_str() );
      }
#endif

      CGparameter get_uniform_param( std::string const &  name ) const
      {
#ifndef NDEBUG
        CGparameter param = cgGetNamedParameter( m_program, name.c_str() );
        if ( param == 0 )
        {
          std::cout << "Program::get_uniform_param(): Uniform variable " << name.c_str() << " was not found or illegal." << std::endl;
          return 0;
        }
        name_type_map::const_iterator lookup = m_uniforms.find( name );
        // TODO: enable again - does not work well with arrays...
        //assert( lookup != m_uniforms.end() || !"Program::get_uniform_param(): The uniform map does not correspond with the Cg runtime." );

        //std::string type = lookup->second;
        //if ( !type.find( "sampler" ) != 0 )
        //{
        //  std::cout << "Program::get_uniform_param(): Uniform variable " << name.c_str() << " is a sampler (texture)." << std::endl;
        //  return 0;
        //}
        return param;
#else
        return cgGetNamedParameter( m_program, name.c_str() );
#endif
      }

      CGparameter get_attribute_param( std::string const & name ) const
      {
#ifndef NDEBUG
        CGparameter param = cgGetNamedParameter( m_program, name.c_str() );
        if ( param == 0 )
        {
          std::cout << "Program::get_attribute_param(): Attribute variable " << name.c_str() << " was not found or illegal." << std::endl;
          return 0;
        }
        return param;
#else
        return cgGetNamedParameter( m_program, name.c_str() );
#endif
      }

    public:

      void set_modelview_projection_matrix(std::string const & name = "ModelViewProj")
      {
        CGparameter param = get_uniform_param( name );
        cgGLSetStateMatrixParameter ( param, CG_GL_MODELVIEW_PROJECTION_MATRIX, CG_GL_MATRIX_IDENTITY );
        gl::gl_check_errors("After Program::set_modelview_projection_matrix()");
      }

      void set_modelview_inverse_transpose(std::string const & name = "ModelViewIT")
      {
        CGparameter param = get_uniform_param( name );
        cgGLSetStateMatrixParameter ( param, CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_INVERSE_TRANSPOSE );
        gl::gl_check_errors("After Program::set_modelview_inverse_transpose()");
      }

      void set_modelview_inverse_matrix(std::string const & name = "ModelViewI")
      {
        CGparameter param = get_uniform_param( name );
        cgGLSetStateMatrixParameter ( param, CG_GL_MODELVIEW_MATRIX, CG_GL_MATRIX_INVERSE );
        gl::gl_check_errors("After Program::set_float_param()");
      }

      void set_texture_matrix(std::string const & name = "TexMatrix")
      {
        CGparameter param = get_uniform_param( name );
        cgGLSetStateMatrixParameter ( param, CG_GL_TEXTURE_MATRIX, CG_GL_MATRIX_IDENTITY );
        gl::gl_check_errors("After Program::set_texture_matrix()");
      }

    public:

      void set_float_param(std::string const & name, float const value)
      {
        CGparameter param = get_uniform_param(name);
        cgGLSetParameter1f(param, value);
        gl::gl_check_errors("After Program::set_float_param()");
      }

      void set_float_param(std::string const & name, float const value0, float const value1)
      {
        CGparameter param = get_uniform_param( name );
        cgGLSetParameter2f(param, value0, value1);
        gl::gl_check_errors("After Program::set_float_param()");
      }

      void set_float_param(std::string const & name, float const value0, float const value1, float const value2)
      {
        CGparameter param = get_uniform_param( name );
        cgGLSetParameter3f(param, value0, value1, value2);
        gl::gl_check_errors("After Program::set_float_param()");
      }

      void set_float_param(std::string const & name, float const value0, float const value1, float const value2, float const value3)
      {
        CGparameter param = get_uniform_param( name );
        cgGLSetParameter4f(param, value0, value1, value2, value3);
        gl::gl_check_errors("After Program::set_float_param()");
      }

      void set_float_array_param(std::string const & name, int const number_of_elements, float const * values)
      {
        CGparameter param = get_uniform_param( name );
        int start_index = 0;
        cgGLSetParameterArray1f(param, start_index, number_of_elements, values);
        gl::gl_check_errors("After Program::set_float_array_param()");
      }

      void set_float3_array_param(std::string const & name, int const number_of_elements, float const * values)
      {
        CGparameter param = get_uniform_param( name );
        int start_index = 0;
        cgGLSetParameterArray3f(param, start_index, number_of_elements, values );
        gl::gl_check_errors("After Program::set_float3_array_param()");
      }

      void set_floatNxN_array_param(std::string const & name, int number_of_elements, float const * values)
      {
        CGparameter param = get_uniform_param( name );
        int start_index = 0;
        cgGLSetMatrixParameterArrayfr( param, start_index, number_of_elements, values );
        gl::gl_check_errors("After Program::set_float4x4_array_param()");
      }

      void enable_client_state(std::string const & name)
      {
        CGparameter param = get_attribute_param( name );
        cgGLEnableClientState(param);
        gl::gl_check_errors("After Program::enable_client_state()");
      }

      void disable_client_state(std::string const & name)
      {
        CGparameter param = get_attribute_param( name );
        cgGLDisableClientState(param);
        gl::gl_check_errors("After Program::disable_client_state()");
      }

    public:

      template<typename texture_pointer>
      void set_input_texture( std::string const & name, texture_pointer texture  )
      {
        texture->bind();
        gl::gl_check_errors("Program::set_input_texture() - texture->bind();");

        CGparameter param = get_texture_param(name, *texture );
        gl::gl_check_errors("Program::set_input_texture() - cgGLEnableTextureParameter( param ) );");

        cgGLSetTextureParameter( param, texture->get_texture_ID() );
        gl::gl_check_errors("Program::set_input_texture() - cgGLSetTextureParameter( param, texture->get_texture_ID() );");

        cgGLEnableTextureParameter(param);
      }

      void set_attribute_pointer( std::string const & name, int size, int type,  unsigned int stride, const void *pointer)
      {
        CGparameter param = get_attribute_param( name );
        cgGLSetParameterPointer(param, size, type, stride, (GLvoid*)(pointer));
        gl::gl_check_errors("Program::set_attribute_pointer()");
      }
    };

  } // namespace cg
} // namespace OpenTissue

// OPENTISSUE_GPU_CG_CG_PROGRAM_H
#endif
