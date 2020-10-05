//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include "OpenTissue/graphics/core/window.h"
#include "OpenTissue/graphics/core/key_events.h"
#include "OpenTissue/graphics/core/mouse_events.h"
#include "OpenTissue/graphics/core/application_events.h"

#include "OpenTissue/graphics/graphics_export.h"

namespace OpenTissue {
namespace graphics {

class GlutWindow;

template<>
class WindowTraits<GlutWindow>
{
public:
  using WindowType   = int;
};

class OT_GRAPHICS_DEPRECATED_API GlutWindow : public Window<GlutWindow>
{
public:
  using Self = GlutWindow;
  using Base = Window<GlutWindow>;

public:
  GlutWindow() = delete;
  GlutWindow(const WindowProperties &properties);

  void add_sub_menu(const std::string &name,
                    const std::unordered_map<unsigned char, const char*> &menu_map);

  void init();

  void update();

  void set_vsync(bool enabled);

  void set_event_callback(const CallBackFnType &fn);

  void shutdown();


private:
  int m_handle;
  int m_main_menu;
};

} // namespace graphics
} // namespace OpenTissue
