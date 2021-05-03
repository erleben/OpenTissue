//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//

#include "OpenTissue/graphics/glut/glut_application.h"

#include <GL/freeglut.h>

#include "OpenTissue/graphics/glut/glut_window.h"
#include "OpenTissue/graphics/core/event.h"


namespace OpenTissue {
namespace graphics {

GlutApplication::GlutApplication(const std::string &title)
    : Application<GlutApplication>(title)
  {
  }

//-------------------------------------------------------------------------------------------

void GlutApplication::idle()
{
  return;
}

//-------------------------------------------------------------------------------------------

void GlutApplication::run()
{
  glutMainLoop();
}

//-------------------------------------------------------------------------------------------

bool GlutApplication::on_event(const Event &)
{
  glutPostRedisplay();
  return true;
}

} // namespace graphics
} // namespace OpenTissue
