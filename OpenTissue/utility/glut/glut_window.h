//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include "OpenTissue/utility/graphics/window.h"
#include "OpenTissue/utility/graphics/key_events.h"
#include "OpenTissue/utility/graphics/mouse_events.h"
#include "OpenTissue/utility/graphics/application_events.h"

namespace OpenTissue {

namespace glut {

class GlutWindow;
class GlutApplication;

}

namespace graphics {

template<>
class WindowTraits<glut::GlutWindow>
{
public:
  using WindowType   = int;
};

}

namespace glut {

class GlutWindow : public graphics::Window<GlutWindow>
{
public:
  using Self = GlutWindow;
  using Base = graphics::Window<GlutWindow>;
  using EventHandler = graphics::EventHandler<glut::GlutApplication>;

public:
  GlutWindow() = delete;
  GlutWindow(const graphics::WindowProperties &properties);

  void add_sub_menu(const std::string &name, 
                    const std::unordered_map<unsigned char, const char*> &menu_map);

  void init();

  void update();

private:
  int m_handle;
  int m_main_menu;
};

}
}
