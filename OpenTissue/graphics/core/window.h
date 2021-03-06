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
#include <string>
#include <unordered_map>

namespace OpenTissue {
namespace graphics {

class Event;
class WindowDisplayEvent;
class WindowProperties;

template<typename T>
class WindowTraits;

template<typename Derived>
class Window
{
public:
  using Self          = Window<Derived>;
  using Ptr           = std::shared_ptr<Self>;
  using WindowTypePtr = typename WindowTraits<Derived>::WindowType;
  using CallBackFnType = std::function<void(const Event&)>;

  template<typename... Args>
  static Ptr New(Args&&... args)
  {
    static auto p = std::make_shared<Derived>(args...);
    return std::static_pointer_cast<Self>(p);
  }

public:
  Window(const Window &)            = delete;
  Window operator=(const Window &)  = delete;
  Window()                          = delete;

  explicit Window(const WindowProperties &properties);
  virtual ~Window();

  void init()
  {
    static_cast<Derived*>(this)->init();
  }

  void update()
  {
    static_cast<Derived*>(this)->update();
  }

  void shutdown()
  {
    static_cast<Derived*>(this)->shutdown();
  }

  void set_vsync(bool enabled)
  {
    static_cast<Derived*>(this)->set_vsync(enabled);
  }

  void set_event_callback(const CallBackFnType &fn)
  {
    m_data->fn = fn;
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

  uint32_t get_width() const
  {
    return m_data->width;
  }

  uint32_t get_height() const
  {
    return m_data->height;
  }

protected:
  struct WindowData;
  std::shared_ptr<WindowData> m_data;
  bool                        m_vsync = false; ///< Boolean flag indicating whether vertical syncronization if active.
  static size_t               m_count;
};

//-------------------------------------------------------------------------------------------------

class WindowProperties
{
public:
  WindowProperties(const std::string& title = "OpenTissue Engine",
                   uint32_t width = 1280,
                   uint32_t height = 720)
    : title(title), width(width), height(height)
  {
  }

  std::string title;
  uint32_t width;
  uint32_t height;

};

//-------------------------------------------------------------------------------------------------

template<typename WindowType>
struct Window<WindowType>::WindowData
{
  std::string title;
  uint32_t width, height;
  Window<WindowType>::CallBackFnType fn;
};

//-------------------------------------------------------------------------------------------------

template<typename WindowType>
Window<WindowType>::Window(const WindowProperties &properties)
  : m_data(std::make_shared<WindowData>())
{
  m_data->title = properties.title;
  m_data->width = properties.width;
  m_data->height = properties.height;
}

//-------------------------------------------------------------------------------------------------

template<typename WindowType>
Window<WindowType>::~Window()
{
}

//-------------------------------------------------------------------------------------------------

template<typename WindowType>
size_t Window<WindowType>::m_count = 0;

} // namespace graphics
} // namespace OpenTissue
