//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <exception>

#include <OpenTissue/configuration.h>
#include "OpenTissue/graphics/glut/glut_window.h"
#include "OpenTissue/graphics/core/application.h"

#include "OpenTissue/graphics/graphics_export.h"

namespace OpenTissue {
namespace graphics {

// Forward declarations
class GlutApplication;
class GlutWindow;
class Event;

template<>
class ApplicationTraits<GlutApplication>
{ 
public:
  using WindowType = GlutWindow;
};

/**
 * A base class for any application based on a GLUT binding.
 *
 * This class defines the basic interface of an application. One uses
 * this base class by making specialized derived types. See for instance
 * the class PerspectiveViewApplication as an example.
 *
 * The class also defines data needed for setting up a window mapping
 * (width and height in pixels) and frustum (near and far clipping
 * planes and fov).
 *
 */
class OT_GRAPHICS_DEPRECATED_API GlutApplication : public Application<GlutApplication>
{
public: 
  GlutApplication(const std::string &title);
  virtual ~GlutApplication() = default;

  bool on_event(const Event &);
  void idle();
  
  virtual void run();
  virtual void shutdown();
  virtual void action(unsigned char choice) = 0;
  virtual void update(double timestep) = 0;
  virtual void init() = 0;
};

} // namespace graphics
} // namespace OpenTissue

OpenTissue::graphics::GlutApplication::Ptr createApplication(int ac, char **av);

#ifdef OT_INJECT_MAIN
  #include "OpenTissue/graphics/core/main.h"
#endif
