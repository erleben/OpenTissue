//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//

#include "OpenTissue/graphics/glfw/glfw_window.h"

#include <string>

#include "OpenTissue/graphics/core/gl/gl.h"
#include "OpenTissue/graphics/core/key_events.h"
#include "OpenTissue/graphics/core/mouse_events.h"
#include "OpenTissue/graphics/core/application_events.h"

namespace OpenTissue {
namespace graphics {

GlfwWindow::GlfwWindow(const WindowProperties &properties)
    : Base(properties)
{
}

//-------------------------------------------------------------------------------------------

void GlfwWindow::update()
{
	glfwPollEvents();
	glfwSwapBuffers(m_handle.get());
}

//-------------------------------------------------------------------------------------------

void GlfwWindow::shutdown()
{
  glfwDestroyWindow(m_handle.get());
  --m_count;

  if(m_count == 0)
  {
    glfwTerminate();
  }
}

//-------------------------------------------------------------------------------------------

void GlfwWindow::set_vsync(bool enabled)
{
  if(enabled)
  {
    glfwSwapInterval(1);
  }
  else
  {
    glfwSwapInterval(0);
  }
  m_vsync = enabled;
}

//-------------------------------------------------------------------------------------------

void GlfwWindow::init()
{
  if(m_count == 0)
  {
	  int success = glfwInit();
    if(!success)
    {
      std::string error("Error: Failed to initialize GLFW.");
      throw std::runtime_error(__PRETTY_FUNCTION__ + error);
    }
    glfwSetErrorCallback([](int error, const char *description){
      std::string message(" :GLFW Error: ");
      throw std::runtime_error(__PRETTY_FUNCTION__ + message + description);
    });
  }

  #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  #endif

  m_handle = std::shared_ptr<GLFWwindow>(
      glfwCreateWindow((int)m_data->width, (int)m_data->height, m_data->title.c_str(), nullptr, nullptr),
      glfwDestroyWindow);
  ++m_count;

  glfwMakeContextCurrent(m_handle.get());
  glfwSetFramebufferSizeCallback(m_handle.get(),[](GLFWwindow* window, int width, int height){
    glViewport(0, 0, width, height);
  });
  glfwSetTime(0.0);

	glfwSetWindowUserPointer(m_handle.get(), m_data.get());
  this->set_vsync(false);

  // Setup event callbacks
  {
    glfwSetWindowSizeCallback(m_handle.get(), [](GLFWwindow *window, int width, int height)
    {
			auto data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
      data->width = width;
      data->height = height;
      WindowResizeEvent e(width, height);
      data->fn(e);
    });

    glfwSetWindowCloseCallback(m_handle.get(), [](GLFWwindow *window)
    {
			auto data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
      WindowCloseEvent e;
      data->fn(e);
    });

    glfwSetKeyCallback(m_handle.get(), [](GLFWwindow* window, int key, int /**scancode*/, int action, int /**mods*/)
    {
			auto data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
      auto code = static_cast<KeyCode>(key);
      switch (action)
			{
				case GLFW_PRESS:
				{
					KeyPressedEvent e(code, 0);
          data->fn(e);
					break;
				}
				case GLFW_RELEASE:
				{
					KeyReleasedEvent e(code);
          data->fn(e);
					break;
				}
				case GLFW_REPEAT:
				{
					KeyPressedEvent e(code, 1);
          data->fn(e);
					break;
				}
			}
    });

		glfwSetCharCallback(m_handle.get(), [](GLFWwindow* window, unsigned int key)
		{
			auto data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
      auto code = static_cast<KeyCode>(key);
			KeyTypedEvent e(code);
      data->fn(e);
		});

		glfwSetMouseButtonCallback(m_handle.get(), [](GLFWwindow* window, int button, int action, int /*mods*/)
		{
			auto data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
      auto code = static_cast<MouseCode>(button);

      double x;
      double y;
      glfwGetCursorPos(window, &x, &y);

      auto shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) & GLFW_PRESS;
      auto ctrl  = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) & GLFW_PRESS;
      auto alt   = glfwGetKey(window, GLFW_KEY_LEFT_ALT) & GLFW_PRESS;

      KeyCode key = KeyCode::None;
      if(shift)
      {
        key = KeyCode::LeftShift;
      }
      else if(ctrl)
      {
        key = KeyCode::LeftControl;
      }
      else if (alt)
      {
        key = KeyCode::LeftAlt;
      }

      switch(action)
      {
        case GLFW_PRESS:
        {
          MouseButtonPressedEvent e(code, key, x, y);
          data->fn(e);
          break;
        }
        case GLFW_RELEASE:
        {
          MouseButtonReleasedEvent e(code, x, y);
          data->fn(e);
          break;
        }
      }
    });

    glfwSetCursorPosCallback(m_handle.get(), [](GLFWwindow* window, double x, double y)
		{
			auto data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
			MouseMovedEvent e(x, y);
      data->fn(e);
		});

		glfwSetScrollCallback(m_handle.get(), [](GLFWwindow* window, double x, double y)
		{
			auto data = static_cast<WindowData*>(glfwGetWindowUserPointer(window));
      MouseScrolledEvent e(x, y, y - x);
      data->fn(e);
		});

  }
}

} // namespace graphics
} // namespace OpenTissue
