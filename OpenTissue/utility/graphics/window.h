//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <unordered_map>
#include <memory>

namespace OpenTissue {
namespace graphics {

template<typename T>
class Event;

template<typename T>
class WindowTraits;

struct WindowProperties
{
  std::string title;
  uint32_t width;
  uint32_t height;

  int argc;
  char **argv;

  WindowProperties(const std::string& title = "OpenTissue Engine",
                   uint32_t width = 1280,
                   uint32_t height = 720,
                   int argc = 0,
                   char **argv = nullptr)
    : title(title), width(width), height(height), argc(argc), argv(argv)
  {
  }
};

template<typename Derived>
class Window
{
public:
  using Self          = Window<Derived>;
  using Ptr           = std::shared_ptr<Self>;
  using WindowTypePtr = typename WindowTraits<Derived>::WindowType;
  // using EventHandler  = typename WindowTraits<Derived>::EventHandler;

public:
  Window(const Window &)            = delete;
  Window operator=(const Window &)  = delete;
  Window()                          = delete;
  virtual ~Window()                 = default;

  explicit Window(const WindowProperties &properties) : m_properties(properties) {}

  template<typename... Args>
  static Ptr New(Args&&... args)
  {
    static auto p = std::make_shared<Derived>(args...);
    return std::static_pointer_cast<Self>(p);
  }

  void init()
  {
    static_cast<Derived*>(this)->init();
  }

  void update()
  {
    static_cast<Derived*>(this)->update();
  }

  void set_vsync(bool enabled)
  {
    static_cast<Derived*>(this)->set_vsync(enabled);
  }

  bool is_vsync() const
  {
    static_cast<Derived*>(this)->is_vsync();
  }

  WindowTypePtr get_window()
  {
    return static_cast<Derived*>(this)->get_window();      
  }

  void add_sub_menu(const std::string &name,
                    std::unordered_map<unsigned char, const char*> menu_map)
  {
    static_cast<Derived*>(this)->add_sub_menu(name, menu_map);
  }

  // void set_event_handler(typename EventHandler::Ptr event_handler)
  // {
  //   m_event_handler = event_handler;
  // }

  // template<typename T>
  // bool event_callback(const Event<T> &e)
  // {
  //   if(m_event_handler)
  //   {
  //     m_event_handler->run(e);
  //   }
  // }

  void set_width(const uint32_t width)
  {
    m_properties.width = width;
  }

  void set_height(const uint32_t height)
  {
    m_properties.height = height;
  }

  uint32_t get_width() const
  {
    return m_properties.width;
  }

  uint32_t get_height() const
  {
    return m_properties.height;
  }

  static void set_event_dispatcher(const std::function<void()> &fn)
  {
    Self::m_event_dispatcher = fn;
  }

  static const std::function<void()> &get_event_dispatcher()
  {
    return Self::m_event_dispatcher;
  }

protected:
  WindowProperties              m_properties;
  static std::function<void()>  m_event_dispatcher;
  bool                       m_vsync;
};

template<typename Derived> 
Window<Derived>::m_event_dispatcher = 

}
}
