//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <functional>
#include <memory>
#include <unordered_map>

// #include "OpenTissue/utility/graphics/event.h"

namespace OpenTissue {
namespace graphics {

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
  using CallBackFnType = std::function<void(const Event&)>;

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

  void set_event_callback(const CallBackFnType &fn)
  {
    static_cast<Derived*>(this)->set_event_callback(fn);
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

protected:
  WindowProperties m_properties;
  bool             m_vsync = false; ///< Boolean flag indicating whether vertical syncronization if active.
};

}
}
