//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "OpenTissue/graphics/core/window.h"
#include "OpenTissue/graphics/graphics_export.h"

namespace OpenTissue {
namespace graphics {

class GlfwWindow;

template<>
class WindowTraits<GlfwWindow>
{
public:
  using WindowType = GLFWwindow;
};

class OT_GRAPHICS_API GlfwWindow : public Window<GlfwWindow>
{
public:
  using Self = GlfwWindow;
  using Base = Window<GlfwWindow>;

public:
  GlfwWindow() = delete;
  GlfwWindow(const WindowProperties &properties);

  void init();

  void update();

  void set_vsync(bool enabled);

  void set_event_callback(const CallBackFnType &fn);

  void shutdown();

private:
  std::shared_ptr<GLFWwindow> m_handle;
};

} // namespace graphics
} // namespace OpenTissue
