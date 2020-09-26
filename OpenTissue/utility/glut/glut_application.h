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
#include "OpenTissue/utility/glut/glut_window.h"
#include "OpenTissue/utility/graphics/application.h"

#include "OpenTissue/utility/graphics_export.h"

namespace OpenTissue {

namespace glut {

class GlutApplication;
class GlutWindow;

}

namespace graphics {
template<>
class ApplicationTraits<glut::GlutApplication>
{ 
public:
  using WindowType = glut::GlutWindow;
};

class Event;

}


namespace glut {

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
class OT_GRAPHICS_DEPRECATED_API GlutApplication : public graphics::Application<GlutApplication>
{
public: 
  GlutApplication(const std::string &title);


  void add_sub_menu(const std::string &name,
                    std::unordered_map<unsigned char, const char*> menu_map);

  void shutdown();

  void run();

  bool on_event(const graphics::Event &);
  
  float get_time() const;

  virtual void action(unsigned char choice) = 0;
  virtual void update(float timestep) = 0;
  virtual void idle() = 0;
  virtual void init() = 0;
  
  
private:
  std::unordered_map<char, std::string> m_menu_map;
};

} // namespace glut
} // namespace OpenTissue

OpenTissue::glut::GlutApplication::Ptr createApplication(int ac, char **av);

#ifdef OT_INJECT_MAIN
  #include "OpenTissue/utility/graphics/main.h"
#endif
