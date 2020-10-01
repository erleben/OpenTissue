#ifndef OPENTISSUE_GPU_CG_UTIL_H
#define OPENTISSUE_GPU_CG_UTIL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/graphics/core/gl/gl_util.h>

// henrikd - cgGL.h includes windows.h, so we just need to help it a little
#ifdef WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#endif
#include <Cg/cg.h>
#include <Cg/cgGL.h>
#ifdef WIN32
#  undef WIN32_LEAN_AND_MEAN
#  undef NOMINMAX
#endif

#include <iostream>
#include <cassert>

namespace OpenTissue
{
  namespace cg
  {

    /**
    * Cg Error Callback function.
    */
    inline static void cg_error_callback_func()
    {
      const char * errorString = cgGetErrorString( cgGetError() );
      std::cout << "Cg Runtime Error encountered" << std::endl;
      std::cout << "\t" << errorString << std::endl;
    }

    class Program;  ///< Forwad declaration


      /**
      * Get the Current CG context.
      */
      inline static CGcontext & context()
      {
        static CGcontext context = 0;
        return context;
      }

      /**
      * Get the current vertex profile.
      */
      inline static CGprofile & vertex_profile()
      {
        static CGprofile vertex_profile;
        return vertex_profile;
      }

      /**
      * Get the current fragment profile.
      */
      inline static CGprofile & fragment_profile()
      {
        static CGprofile fragment_profile;
        return fragment_profile;
      }

      /**
      * Get the current vertex program.
      */
      inline static Program * & vertex_program()
      {
        static Program * v_program = 0;    ///<
        return v_program;
      }

      /**
      * Get the current fragment program.
      */
      inline static Program * & fragment_program()
      {
        static Program * f_program = 0;    ///<
        return f_program;
      }

      /**
      * Invoke this function just after having created a
      * GL context but before doing anything else.
      *
      * This function will start up Cg and initialize
      * any neccessary settings such as profiles.
      */
      inline static bool startup()
      {
        bool startup_succeded = true;
        cgSetErrorCallback( cg_error_callback_func );
        context() = cgCreateContext();

        cgGLSetManageTextureParameters(context(),true);

        if(context()==0)
        {
          std::cerr << "Cg::startup(): Could not create context" << std::endl;
          startup_succeded = false;
        }

        vertex_profile() = cgGLGetLatestProfile(CG_GL_VERTEX);
        if(vertex_profile() == CG_PROFILE_UNKNOWN)
        {
          std::cerr << "Cg::startup(): invalid vertex profile" << std::endl;
          startup_succeded = false;
        }
        else
        {
          cgGLSetOptimalOptions( vertex_profile() );
        }

        std::cout << "vertex profile = " << cgGetProfileString(vertex_profile()) << std::endl;

        fragment_profile() = cgGLGetLatestProfile(CG_GL_FRAGMENT);

        if(fragment_profile() == CG_PROFILE_UNKNOWN)
        {
          std::cerr << "Cg::startup(): invalid fragment profile" << std::endl;
          startup_succeded = false;
        }
        else
        {
          cgGLSetOptimalOptions( fragment_profile() );
        }

        std::cout << "fragment profile = " << cgGetProfileString(fragment_profile()) << std::endl;

        return startup_succeded;
      }

      /**
      * Before closing an application invoke this
      * method to close down Cg in nice and proper manner.
      */
      inline static void shutdown()
      {
        cgGLDisableProfile( vertex_profile() );
        cgGLDisableProfile( fragment_profile() );
        cgDestroyContext( context() );
        context() = 0;
      }

  } // namespace cg
} // namespace OpenTissue

// OPENTISSUE_GPU_CG_UTIL_H
#endif
