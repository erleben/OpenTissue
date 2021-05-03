//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//

#include "OpenTissue/graphics/glfw/glfw_application.h"

#include "OpenTissue/graphics/glfw/glfw_window.h"
#include "OpenTissue/graphics/core/event.h"

namespace OpenTissue {
namespace graphics {

GlfwApplication::GlfwApplication(const std::string &title)
    : Application<GlfwApplication>(title)
  {
  }

//-------------------------------------------------------------------------------------------

void GlfwApplication::idle()
{
}

//-------------------------------------------------------------------------------------------

bool GlfwApplication::on_event(const Event &)
{
  return true;
}

} // namespace graphics
} // namespace OpenTissue
