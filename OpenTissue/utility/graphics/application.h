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

#include "OpenTissue/utility/gl/gl.h"

#include "OpenTissue/core/math/math_basic_types.h"
#include "OpenTissue/utility/graphics/event.h"
#include "OpenTissue/utility/graphics/application_events.h"
#include "OpenTissue/utility/graphics/key_events.h"
#include "OpenTissue/utility/graphics/mouse_events.h"
#include "OpenTissue/utility/graphics/window.h"
#include "OpenTissue/utility/gl/gl_camera.h"
#include "OpenTissue/utility/gl/gl_frustum.h"

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
  using Self           = Application<Derived>;
  using Ptr            = std::shared_ptr<Self>;
  using MathTypes      = OpenTissue::math::default_math_types;
  using CameraType     = gl::Camera<MathTypes>;  ///< Camera class taking care of the model-view projection transformation.
  using FrustumType    = gl::Frustum<MathTypes>; ///< Frustum class, can be used for optimizing the rendering process by offering simple frustum culling.
  using WindowType     = typename ApplicationTraits<Derived>::WindowType;   ///< Window

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
  Application() = delete;
  Application(const Application&) = delete;
  Application(Application&&) = delete;
  Application& operator=(const Application&) = delete;
  Application& operator=(Application&&) = delete;

  virtual ~Application() = default;
  
  void shutdown()
  {
    static_cast<Derived*>(this)->shutdown();
  }

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
					this->display(time);
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
    // auto event_handler = EventHandler<Self>::New();
    m_window = WindowType::New(properties);
    // m_window->set_event_handler(event_handler);
  }

  void add_sub_menu(const std::string &name,
                    const std::unordered_map<unsigned char, std::string> &menu_map)
  {
    static_cast<Derived*>(this)->add_sub_menu(name, menu_map);    
  }

  CameraType &camera()
  {
    return m_camera;
  }

  const CameraType &camera() const
  {
    return m_camera;
  }

private:
  float get_time() const
  {
    return static_cast<const Derived*>(this)->get_time();
  }

  void display(float timestep)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m_camera.get_modelview_matrix());
    m_frustum.update();
    
    static_cast<Derived*>(this)->update(timestep);
    glFinish();
    glutSwapBuffers();
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
    static_cast<Derived*>(this)->init();
  }

  void init_gl()
  {
    auto err = glewInit();
    if (GLEW_OK != err)
    {
      // auto err_string = std::string(glewGetErrorString(err));
      throw std::runtime_error("Application::init_gl() - GLEW Error: " /* + err_string */);
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

public: // Event handling
  void on_event(const Event &e)
  {
    EventType type = e.get_event_type();
    switch(type)
    {
      case EventType::WindowClose:
      {
        this->on_window_close(static_cast<const WindowCloseEvent&>(e));
        break;
      }
      case EventType::WindowResize:
      {
        this->on_window_resize(static_cast<const WindowResizeEvent&>(e));
        break;
      }
      case EventType::WindowDisplay:
      {
        this->on_window_display(static_cast<const WindowDisplayEvent&>(e));
        break;
      }
      case EventType::KeyPressed:
      {
        this->on_key_pressed(static_cast<const KeyPressedEvent&>(e));
        break;
      }
      case EventType::KeyReleased:
      {
        break;
      }
      case EventType::KeyTyped:
      {
        break;
      }
      case EventType::MouseButtonPressed:
      {
        this->on_mouse_button_pressed(static_cast<const MouseButtonPressedEvent&>(e));
        break;
      }
      case EventType::MouseButtonReleased:
      {
        this->on_mouse_button_released(static_cast<const MouseButtonReleasedEvent&>(e));
        break;
      }
      case EventType::MouseMoved:
      {
        this->on_mouse_moved(static_cast<const MouseMovedEvent&>(e));
        break;
      }
      case EventType::MouseScrolled:
      {
        break;
      }
    };
  }
  
  //----------------------------------------------------------------------------

  bool on_window_display(const WindowDisplayEvent &e)
  {
    this->display(this->get_time());
    static_cast<Derived*>(this)->on_event(e);
    return true;
  }
  
  //----------------------------------------------------------------------------

  bool on_window_close(const WindowCloseEvent &e)
  {
    m_running = false;
    static_cast<Derived*>(this)->on_event(e);
    return true;
  }

  //----------------------------------------------------------------------------

  bool on_window_resize(const WindowResizeEvent &e)
  {
    auto width = e.get_width();
    auto height = e.get_height();

    m_window->set_width(width);
    m_window->set_height(height);

    if (width == 0 || height == 0)
    {
      m_minimized = true;
      return false;
    }
    
    m_minimized = false;
    glViewport(0, 0, width, height);

    // Update modelview
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    m_aspect = width / height;
    std::cout << m_aspect << std::endl;
    gluPerspective(m_fovy, m_aspect, m_z_near, m_z_far);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    this->update_lights();
    static_cast<Derived*>(this)->on_event(e);

    return true;
  }

  //----------------------------------------------------------------------------

  bool on_key_pressed(const KeyPressedEvent &e)
  {
    auto key = static_cast<unsigned char>(e.get_code());
    switch (key)
    {
      case 'q':
        this->shutdown();
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
    };

    static_cast<Derived*>(this)->action(key);
    static_cast<Derived*>(this)->on_event(e);

    return true;
  }

  //----------------------------------------------------------------------------

  bool on_mouse_button_pressed(const MouseButtonPressedEvent &e)
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

    if (mod & KeyCode::LeftShift && (button == MouseCode::ButtonLeft))
    {
      m_pan_mode = true;
    }

    if(!(mod & KeyCode::LeftControl) &&  
       !(mod & KeyCode::LeftAlt) && 
       !(mod & KeyCode::LeftShift) && 
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

    return true;
  }

  //----------------------------------------------------------------------------

  bool on_mouse_button_released(const MouseButtonReleasedEvent &e)
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

    return true;
  }

  //----------------------------------------------------------------------------

  bool on_mouse_moved(const MouseMovedEvent &e)
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

    return true;
  }

protected:
  typename WindowType::Ptr m_window;
	friend int ::main(int argc, char** argv);  ///< Run loop is only accessible from main.
  CameraType	  m_camera;                    ///< Camera class taking care of the model-view projection transformation.
  FrustumType	  m_frustum;                   ///< Frustum class, can be used for optimizing the rendering process by offering simple frustum culling.

  double        m_fovy              = 30.0;  ///< Field of view in the y-direction.
  double        m_aspect            = 1.0;   ///< Aspect ratio.
  double        m_z_near            = 0.1;   ///< Near clipping plane.
  double        m_z_far             = 700.0; ///< Far clipping plane.
  double        m_zoom_sensitivity  = 0.25;  ///< The zooming mouse sensitivity.
  double        m_pan_sensitivity   = 0.25;  ///< The panning mouse sensitivity.
  bool          m_screen_capture    = false; ///< Boolean flag indicating whether a screen capture should be performing on the next display event.
  bool          m_idle_on           = false; ///< Boolean flag indicating whether the idle function is on or off.
	bool          m_running           = true;  ///< Loop control variable
	bool          m_minimized         = false; ///< Loop control variable
  bool          m_trackball_mode    = false; ///< Boolean flag indicating whether the application is currently doing a trackball operation.
  bool          m_zoom_mode         = false; ///< Boolean flag indicating whether the application is currently doing a zoom operation.
  bool          m_pan_mode          = false; ///< Boolean flag indicating whether the application is currently doing a pan operation.
  float         m_begin_x           = 0.0f;  ///< The starting x-pixel coordinate when doing a mouse operation.
  float         m_begin_y           = 0.0f;  ///< The starting y-pixel coordinate when doing a mouse operation.
};

} // namespace graphics

} // namespace OpenTissue

