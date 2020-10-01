//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/graphics/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN

class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
public:

  Application()
  {
    //--- This is how one could change the default size of the main window
    this->width() = 512;
    this->height() = 512;
  }

public:

  char const * do_get_title() const { return "My Minimal Application"; }

  void do_display()
  {
    OpenTissue::gl::DrawFrame(math_types::vector3_type(), math_types::quaternion_type() );
  }

  void do_action(unsigned char choice){}

  void do_init_right_click_menu(int main_menu, void menu(int entry)){}

  void do_init(){}

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
