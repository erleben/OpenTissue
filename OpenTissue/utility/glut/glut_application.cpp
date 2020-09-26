//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//

#include "OpenTissue/utility/glut/glut_window.h"
#include "OpenTissue/utility/glut/glut_application.h"
#include "OpenTissue/utility/graphics/event.h"

namespace OpenTissue {
namespace glut {

GlutApplication::GlutApplication(const std::string &title) 
    : graphics::Application<GlutApplication>(title)
  {
  }

//-------------------------------------------------------------------------------------------

void GlutApplication::add_sub_menu(const std::string &name,
                                   std::unordered_map<unsigned char, const char*> menu_map)
{
  if(m_window)
  {
    m_window->add_sub_menu(name, menu_map);
  }
}

//-------------------------------------------------------------------------------------------

void GlutApplication::shutdown()
{
  exit(0);  
}

//-------------------------------------------------------------------------------------------

float GlutApplication::get_time() const
{
  return 0.0;
}

//-------------------------------------------------------------------------------------------

void GlutApplication::run()
{
  glutMainLoop();
}

//-------------------------------------------------------------------------------------------

bool GlutApplication::on_event(const graphics::Event &)
{
  glutPostRedisplay();
  return true;
}

}
}
