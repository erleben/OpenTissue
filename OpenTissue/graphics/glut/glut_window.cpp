//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//

#include "OpenTissue/graphics/glut/glut_window.h"

#include "OpenTissue/graphics/core/gl/gl.h"

namespace OpenTissue {
namespace graphics {

GlutWindow::GlutWindow(const WindowProperties &properties)
    : Base(properties), m_handle(-1), m_main_menu(-1)
{}

//-------------------------------------------------------------------------------------------

void GlutWindow::shutdown()
{
  exit(0);
}

//-------------------------------------------------------------------------------------------

void GlutWindow::update()
{
  glFinish();
  glutSwapBuffers();
}

//-------------------------------------------------------------------------------------------

void GlutWindow::add_sub_menu(const std::string &name,
                              const std::unordered_map<unsigned char, const char*> &menu_map)
{
  auto menu_callback = [](int k)
  {
		auto data = static_cast<WindowData*>(glutGetWindowData());
    KeyPressedEvent e(static_cast<KeyCode>(k), 0);
    if(data->fn)
    {
      data->fn(e);
    }
  };

  if(m_main_menu < 0)
  {
    m_main_menu = glutCreateMenu(menu_callback);
  }

  auto sub_menu = glutCreateMenu(menu_callback);
  for (const auto &entry : menu_map)
  {
    glutAddMenuEntry(entry.second, entry.first);
  }
  glutSetMenu(m_main_menu);
  glutAddSubMenu(name.c_str(), sub_menu );
}

//-------------------------------------------------------------------------------------------

void GlutWindow::init()
{
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(m_data->width, m_data->height);
  glutInitWindowPosition(50, 50);
  m_handle = glutCreateWindow(m_data->title.c_str());

  // Setup menu
  {
    std::unordered_map<unsigned char, const char*> menu_map;
    menu_map.insert(std::make_pair('q', "quit                         [q]"));
    menu_map.insert(std::make_pair(' ', "toggle idle                  [ ]"));
    menu_map.insert(std::make_pair('o', "toggle camera orbit/rotate   [o]"));
    menu_map.insert(std::make_pair('l', "toggle camera target locked  [l]"));
    menu_map.insert(std::make_pair('y', "screen capture               [y]"));

    this->add_sub_menu("controls", menu_map);

    glutSetMenu(m_main_menu);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
  }

  glutSetWindowData(m_data.get());

  // Setup event callbacks
  {
    glutReshapeFunc([](int width, int height)
    {
			auto data = static_cast<WindowData*>(glutGetWindowData());
      data->width = width;
      data->height = height;
      WindowResizeEvent e(width, height);
      if(data->fn)
      {
        data->fn(e);
      }
    });

    glutKeyboardFunc([](unsigned char key, int x, int y)
    {
			auto data = static_cast<WindowData*>(glutGetWindowData());
      auto code = static_cast<KeyCode>(key);
      KeyPressedEvent e(code);
      if(data->fn)
      {
        data->fn(e);
      }
    });

    glutMouseFunc([](int button, int state, int x, int y)
    {
			auto data = static_cast<WindowData*>(glutGetWindowData());
      auto mods = glutGetModifiers();

      KeyCode key;
      switch(mods)
      {
        case GLUT_ACTIVE_SHIFT:
        {
          key = KeyCode::LeftShift;
          break;
        }
        case GLUT_ACTIVE_ALT:
        {
          key = KeyCode::LeftAlt;
          break;
        }
        case GLUT_ACTIVE_CTRL:
        {
          key = KeyCode::LeftControl;
          break;
        }
        default:
        {
          key = KeyCode::None;
        }
      }

      MouseCode code;
      switch(button)
      {
        case GLUT_LEFT_BUTTON:
        {
          code = MouseCode::ButtonLeft;
          break;
        }
        case GLUT_MIDDLE_BUTTON:
        {
          code = MouseCode::ButtonMiddle;
          break;
        }
        case GLUT_RIGHT_BUTTON:
        {
          code = MouseCode::ButtonRight;
          break;
        }
      };

      bool down = (state == GLUT_DOWN);
      if(down)
      {
        MouseButtonPressedEvent e(code, key, x, y);
        if(data->fn)
        {
          data->fn(e);
        }
      }
      else
      {
        MouseButtonReleasedEvent e(code, x, y);
        if(data->fn)
        {
          data->fn(e);
        }
      }
    });

    glutPassiveMotionFunc([](int x, int y)
    {
			auto data = static_cast<WindowData*>(glutGetWindowData());
      MouseMovedEvent e(x, y);
      if(data->fn)
      {
        data->fn(e);
      }
    });

    glutMotionFunc([](int x, int y)
    {
			auto data = static_cast<WindowData*>(glutGetWindowData());
      MouseMovedEvent e(MouseCode::Button0, x, y);
      if(data->fn)
      {
        data->fn(e);
      }
    });

    glutDisplayFunc([]()
    {
			auto data = static_cast<WindowData*>(glutGetWindowData());
      WindowDisplayEvent e;
      if(data->fn)
      {
        data->fn(e);
      }
    });

    glutMouseWheelFunc([](int /*wheel*/, int direction, int /*x*/, int /*y*/)
    {
			auto data = static_cast<WindowData*>(glutGetWindowData());
      MouseScrolledEvent e(0, 0, direction);
      if(data->fn)
      {
        data->fn(e);
      }
    });
  }
}

} // namespace graphics
} // namespace OpenTissue
