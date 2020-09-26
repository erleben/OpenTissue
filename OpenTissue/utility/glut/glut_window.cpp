//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//

#include "OpenTissue/utility/glut/glut_window.h"

#include "OpenTissue/utility/gl/gl.h"
#include "OpenTissue/utility/glut/glut_application.h"

namespace OpenTissue {
namespace glut {

GlutWindow::GlutWindow(const graphics::WindowProperties &properties) 
    : Base(properties), m_handle(-1), m_main_menu(-1)
{}

//-------------------------------------------------------------------------------------------

void GlutWindow::update()
{

}

//-------------------------------------------------------------------------------------------

void GlutWindow::add_sub_menu(const std::string &name, 
                              const std::unordered_map<unsigned char, const char*> &menu_map)
{
  auto menu_callback = [](int k)
  {
    graphics::KeyPressedEvent e(static_cast<graphics::KeyCode>(k), 0);
    if(GlutWindow::m_event_dispatcher)
    {
      GlutWindow::m_event_dispatcher(e);
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
  glutInitWindowSize(m_properties.width, m_properties.height);
  glutInitWindowPosition(50, 50);
  m_handle = glutCreateWindow(m_properties.title.c_str());

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
  
  // Setup event callbacks
  {
    glutReshapeFunc([](int width, int height)
    {
      graphics::WindowResizeEvent e(width, height);
      if(GlutWindow::m_event_dispatcher)
      {
        GlutWindow::m_event_dispatcher(e);
      }
    });
    
    glutKeyboardFunc([](unsigned char key, int x, int y)
    {
      auto code = static_cast<graphics::KeyCode>(key);
      graphics::KeyPressedEvent e(code);
      if(GlutWindow::m_event_dispatcher)
      {
        GlutWindow::m_event_dispatcher(e);
      }
    });

    glutMouseFunc([](int button, int state, int x, int y)
    {
      auto mods = glutGetModifiers();

      int key = 0;
      if(mods & GLUT_ACTIVE_SHIFT)
      {
        key |= graphics::KeyCode::LeftShift;
      }

      if(mods & GLUT_ACTIVE_ALT)
      {
        key |= graphics::KeyCode::LeftAlt;
      }

      if(mods & GLUT_ACTIVE_CTRL)
      {
        key |= graphics::KeyCode::LeftControl;
      }

      graphics::MouseCode code;
      switch(button)
      {
        case GLUT_LEFT_BUTTON:
        {
          code = graphics::MouseCode::ButtonLeft;
          break;
        }
        case GLUT_MIDDLE_BUTTON:
        {
          code = graphics::MouseCode::ButtonMiddle;
          break;
        }
        case GLUT_RIGHT_BUTTON:
        {
          code = graphics::MouseCode::ButtonRight;
          break;
        }
      };

      bool down = (state == GLUT_DOWN);
      if(down)
      {
        graphics::MouseButtonPressedEvent e(code, key, x, y);
        if(GlutWindow::m_event_dispatcher)
        {
          GlutWindow::m_event_dispatcher(e);
        }
      }
      else
      {
        graphics::MouseButtonReleasedEvent e(code, x, y);
        if(GlutWindow::m_event_dispatcher)
        {
          GlutWindow::m_event_dispatcher(e);
        }
      }
    });

    glutPassiveMotionFunc([](int x, int y)
    {
      graphics::MouseMovedEvent e(x, y);
      if(GlutWindow::m_event_dispatcher)
      {
        GlutWindow::m_event_dispatcher(e);
      }
    });

    glutMotionFunc([](int x, int y)
    {
      graphics::MouseMovedEvent e(graphics::MouseCode::Button0, x, y);
      if(GlutWindow::m_event_dispatcher)
      {
        GlutWindow::m_event_dispatcher(e);
      }
    });

    glutDisplayFunc([]()
    {
      graphics::WindowDisplayEvent e;
      if(GlutWindow::m_event_dispatcher)
      {
        GlutWindow::m_event_dispatcher(e);
      }
    });
  }
}

void GlutWindow::set_event_callback(const GlutWindow::CallBackFnType &fn)
{
  GlutWindow::m_event_dispatcher = fn;
}

GlutWindow::CallBackFnType GlutWindow::m_event_dispatcher;
}
}

