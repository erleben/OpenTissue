//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <memory>
#include <string>

#include "OpenTissue/core/math/math_basic_types.h"

#include "OpenTissue/graphics/core/application_events.h"
#include "OpenTissue/graphics/core/event.h"
#include "OpenTissue/graphics/core/event_dispatcher.h"
#include "OpenTissue/graphics/core/gl/gl_camera.h"
#include "OpenTissue/graphics/core/gl/gl_frustum.h"
#include "OpenTissue/graphics/core/gl/gl.h"
#include "OpenTissue/graphics/core/key_events.h"
#include "OpenTissue/graphics/core/mouse_events.h"
#include "OpenTissue/graphics/core/window.h"

int main(int argc, char** argv);

namespace OpenTissue {
namespace graphics {

template<typename T>
class ApplicationTraits;

/**
 * A CRTP based base class for any application.
 *
 * This class defines the basic interface of an application. Use
 * this base class by making specialized derived types.
 *
 */
template<typename Derived>
class Application
{
public:
  using Self        = Application<Derived>;
  using Ptr         = std::shared_ptr<Self>;
  using MathTypes   = OpenTissue::math::default_math_types;
  using CameraType  = gl::Camera<MathTypes>;  ///< Camera class taking care of the model-view projection transformation.
  using FrustumType = gl::Frustum<MathTypes>; ///< Frustum class, can be used for optimizing the rendering process by offering simple frustum culling.
  using WindowType  = typename ApplicationTraits<Derived>::WindowType;   ///< Window

  template<typename ChildType, typename... Args>
  static Ptr New(Args&&... args)
  {
    static auto p = std::static_pointer_cast<Self>(
        std::make_shared<ChildType>(args...));

    return p;
  }

  template<typename... Args>
  static Ptr New(Args&&... args)
  {
    return Self::New<Derived>(args...);
  }

public:
  Application& operator=(const Application&)  = delete;
  Application& operator=(Application&&)       = delete;
  Application(const Application&)             = delete;
  Application(Application&&)                  = delete;
  Application()                               = delete;

  virtual ~Application()                      = default;

  virtual void run()
  {
    float last_frame_time = 0;
    while (m_running)
		{
			float time = this->get_time();
			float timestep = time - last_frame_time;
			last_frame_time = time;

			if (!m_minimized && !m_idle_on)
			{
					this->update(time);
			}

			m_window->update();
		}
  }

protected:
  Application(const WindowProperties &properties)
  {
    this->set_window(properties);
  }

  Application(const std::string &name)
  {
    this->set_window(WindowProperties(name));
  }

  void set_window(const WindowProperties &properties)
  {
    m_window = WindowType::New(properties);
  }

  void add_sub_menu(const std::string &name,
                    const std::unordered_map<unsigned char, const char*> &menu_map)
  {
    if(m_window)
    {
      m_window->add_sub_menu(name, menu_map);
    }
  }

  CameraType &camera()
  {
    return m_camera;
  }

  const CameraType &camera() const
  {
    return m_camera;
  }

  double get_time() const
  {
    return m_time;
  }

private:
  void update(double time)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m_camera.get_modelview_matrix());
    m_frustum.update();

    static_cast<Derived*>(this)->update(time);
    m_window->update();
  }

  void init()
  {
    m_window->set_event_callback([this](const Event &e)
    {
      if(!e.handled())
      {
        this->on_event(e);
      }
    });
    m_window->init();

    this->init_gl();
    this->init_camera();

    // bind events
    m_dispatcher.subscribeListeners(this);

    static_cast<Derived*>(this)->init();
  }

  void init_gl()
  {
    auto err = glewInit();
    if (GLEW_OK != err)
    {
      std::string err_string = reinterpret_cast<const char*>(glewGetErrorString(err));
      throw std::runtime_error("Application::init_gl() - GLEW Error: " + err_string);
    }
    std::cout << "GLEW status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;

    glClearColor(.7, .7, .7, 1.0);
    glEnable(GL_DEPTH_TEST);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    // Shading Model
    glShadeModel(GL_SMOOTH);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

    // Enable Lights
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    this->update_lights();

    // Update othoprojection
    this->update_projection(m_window->get_width(), m_window->get_height());
  }

  void init_camera()
  {
    m_camera.target_locked() = true;
    MathTypes::vector3_type position(0,0,100);
    MathTypes::vector3_type target(0,0,0);
    MathTypes::vector3_type up(0,1,0);
    m_camera.init(position, target, up);
  }

  void update_lights()
  {
    GLfloat ambient_light[]  = { .5, .5, .5, 1.0 };
    GLfloat diffuse_light[]  = { 1., 1., 1., 1.0 };
    GLfloat specular_light[] = { 1.0, 1.0, 1.0, 1.0 };

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular_light);

    // light_position is NOT a default value
    // 4th component == 1 means at finite position,
    //               == 0 means at infinity
    GLfloat light_position[] = { 0.0, 0.0, 1.0, 0. };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  }

  void update_projection(unsigned int width, unsigned int height)
  {
    glViewport(0, 0, width, height);

    // Update modelview
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    m_aspect = width / height;
    gluPerspective(m_fovy, m_aspect, m_z_near, m_z_far);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }

  // Event handling
  void on_event(const Event &e)
  {
    m_dispatcher.post(e);
  }

public:
  //----------------------------------------------------------------------------

  void on_window_display(const WindowDisplayEvent &e)
  {
    this->update(m_time);
    static_cast<Derived*>(this)->on_event(e);
  }

  //----------------------------------------------------------------------------

  void on_window_close(const WindowCloseEvent &e)
  {
    m_running = false;
    static_cast<Derived*>(this)->on_event(e);
  }

  //----------------------------------------------------------------------------

  void on_window_resize(const WindowResizeEvent &e)
  {
    double width = e.get_width();
    double height = e.get_height();

    if (width == 0 || height == 0)
    {
      m_minimized = true;
      return;
    }

    m_minimized = false;

    this->update_projection(width, height);
    this->update_lights();
    static_cast<Derived*>(this)->on_event(e);
  }

  //----------------------------------------------------------------------------

  void on_key_pressed(const KeyPressedEvent &e)
  {
    auto key = static_cast<unsigned char>(e.get_code());
    switch (key)
    {
      case 'q':
        m_window->shutdown();
        m_running = false;
        break;
      case ' ':
        m_idle_on = !m_idle_on;
        static_cast<Derived*>(this)->idle();
        break;
      case 'o':
        m_camera.orbit_mode() = !m_camera.orbit_mode();
        break;
      case 'l':
        m_camera.target_locked() = !m_camera.target_locked();
        break;
      case 'y':
        m_screen_capture = true;
        break;
      default:
        static_cast<Derived*>(this)->action(key);
    };

    static_cast<Derived*>(this)->on_event(e);
  }

  //----------------------------------------------------------------------------

  void on_mouse_button_pressed(const MouseButtonPressedEvent &e)
  {
    auto x      = e.get_x();
    auto y      = e.get_y();
    auto mod    = e.get_modifier();
    auto button = e.get_mouse_button();

    if(button == MouseCode::ButtonMiddle ||
      (button == MouseCode::ButtonLeft &&
       mod == KeyCode::LeftAlt))
    {
      m_zoom_mode = true;
    }

    if (mod == KeyCode::LeftShift && button == MouseCode::ButtonLeft)
    {
      m_pan_mode = true;
    }

    if(!(mod == KeyCode::LeftControl) &&
       !(mod == KeyCode::LeftAlt) &&
       !(mod == KeyCode::LeftShift) &&
       !(button == MouseCode::ButtonMiddle) &&
       !(button == MouseCode::ButtonRight) &&
       button == MouseCode::ButtonLeft) // only left button
    {
      m_camera.mouse_down(x, y);
      m_trackball_mode = true;
    }

    m_begin_x = x;
    m_begin_y = y;

    static_cast<Derived*>(this)->on_event(e);
  }

  //----------------------------------------------------------------------------

  void on_mouse_button_released(const MouseButtonReleasedEvent &e)
  {
    auto x = e.get_x();
    auto y = e.get_y();

    if(m_zoom_mode)
    {
      m_camera.move(m_zoom_sensitivity * (y - m_begin_y));
      m_zoom_mode = false;
    }
    else if(m_pan_mode)
    {
      m_camera.pan(m_pan_sensitivity * (m_begin_x - x) , m_pan_sensitivity * (y - m_begin_y));
      m_pan_mode = false;
    }
    else if(m_trackball_mode)
    {
      m_camera.mouse_up(x, y);
      m_trackball_mode = false;
    }
    m_begin_x = x;
    m_begin_y = y;

    static_cast<Derived*>(this)->on_event(e);
  }

  //----------------------------------------------------------------------------

  void on_mouse_moved(const MouseMovedEvent &e)
  {
    auto x = e.get_x();
    auto y = e.get_y();

    if(m_zoom_mode)
    {
      m_camera.move(m_zoom_sensitivity * (y - m_begin_y));
    }
    else if(m_pan_mode)
    {
      m_camera.pan(m_pan_sensitivity * (m_begin_x - x) , m_pan_sensitivity * (y - m_begin_y));
    }
    else if (m_trackball_mode)
    {
      m_camera.mouse_move(x, y);
    }
    m_begin_x = x;
    m_begin_y = y;

    static_cast<Derived*>(this)->on_event(e);
  }

  //----------------------------------------------------------------------------

  void on_mouse_scrolled(const MouseScrolledEvent &e)
  {
    m_camera.move(100 * m_zoom_sensitivity * e.get_direction());
    static_cast<Derived*>(this)->on_event(e);
  }

protected:
  typename WindowType::Ptr m_window;
	friend int ::main(int argc, char** argv);  ///< Run loop is only accessible from main.
  CameraType	    m_camera;                  ///< Camera class taking care of the model-view projection transformation.
  FrustumType	    m_frustum;                 ///< Frustum class, can be used for optimizing the rendering process by offering simple frustum culling.
  EventDispatcher m_dispatcher;              ///< Event handling

  double          m_fovy              = 30.0;  ///< Field of view in the y-direction.
  double          m_aspect            = 1.0;   ///< Aspect ratio.
  double          m_z_near            = 0.1;   ///< Near clipping plane.
  double          m_z_far             = 700.0; ///< Far clipping plane.
  double          m_zoom_sensitivity  = 0.25;  ///< The zooming mouse sensitivity.
  double          m_pan_sensitivity   = 0.25;  ///< The panning mouse sensitivity.
  bool            m_screen_capture    = false; ///< Boolean flag indicating whether a screen capture should be performing on the next display event.
  bool            m_idle_on           = false; ///< Boolean flag indicating whether the idle function is on or off.
	bool            m_running           = true;  ///< Loop control variable
	bool            m_minimized         = false; ///< Loop control variable
  bool            m_trackball_mode    = false; ///< Boolean flag indicating whether the application is currently doing a trackball operation.
  bool            m_zoom_mode         = false; ///< Boolean flag indicating whether the application is currently doing a zoom operation.
  bool            m_pan_mode          = false; ///< Boolean flag indicating whether the application is currently doing a pan operation.
  float           m_begin_x           = 0.0f;  ///< The starting x-pixel coordinate when doing a mouse operation.
  float           m_begin_y           = 0.0f;  ///< The starting y-pixel coordinate when doing a mouse operation.
  double          m_time              = 0.0;   ///< Simulation time
};

} // namespace graphics
} // namespace OpenTissue
