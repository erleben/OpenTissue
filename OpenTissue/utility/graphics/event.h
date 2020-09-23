//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <memory>

namespace OpenTissue {
namespace graphics {

enum class EventType 
{
  None = 0,
  WindowClose, WindowResize, WindowDisplay,                             // Window events
  KeyPressed, KeyReleased, KeyTyped,                                    // Keyboard events
  MouseButtonPressed, MouseButtonReleased, MouseMoved, MouseScrolled    // Mouse events
};

enum class EventCategory
{
  None        = 0,
  Application = 1 << 0,
  Input       = 1 << 1,
  Keyboard    = 1 << 2,
  Mouse       = 1 << 3,
  MouseButton = 1 << 4
};

template<typename Derived>
class Event
{
public:
  using Self = Event<Derived>;
  using Ptr  = std::shared_ptr<Self>;

public:
  Event() = default;
  virtual ~Event() = default;

  constexpr EventType get_event_type() const
  {
    return static_cast<const Derived*>(this)->get_event_type();
  }

  constexpr EventCategory get_category_flag() const
  {
    return static_cast<const Derived*>(this)->get_category_flag();
  }

  constexpr char* get_name() const
  {
    return static_cast<const Derived*>(this)->get_name();
  }
  
  inline bool is_handled()
  {
    return this->m_handled;
  }

  inline float get_x() const 
  { 
    return static_cast<const Derived*>(this)->get_x(); 
  }

  inline float get_y() const 
  { 
    return static_cast<const Derived*>(this)->get_y(); 
    }

  inline bool is_passive() const 
  { 
    return static_cast<const Derived*>(this)->is_passive(); 
  }  

  inline int get_modifier() const 
  { 
    return static_cast<const Derived*>(this)->get_modifier(); 
  }

  inline float get_xoffset() const 
  { 
    return static_cast<const Derived*>(this)->get_xoffset(); 
  }

  inline float get_yoffset() const 
  { 
    return static_cast<const Derived*>(this)->get_yoffset(); 
  }

  unsigned int get_width() const 
  { 
    return static_cast<const Derived*>(this)->get_width(); 
  }
  
  unsigned int get_height() const 
  { 
    return static_cast<const Derived*>(this)->get_height(); 
  }

};

template<typename EventDispatcher>
class EventHandler final
{
public:  
  using Self = EventHandler<EventDispatcher>;
  using Ptr  = std::shared_ptr<Self>;
  
  template<typename... Args>
  static Ptr New(Args&&... args)
  {
    static Ptr p = std::make_shared<Self>(args...);
    return p;
  }

public:
  explicit EventHandler(typename EventDispatcher::Ptr dispatcher) 
    : m_dispatcher(dispatcher) { }

  EventHandler() : m_dispatcher(EventDispatcher::New()) {}

  template<typename... Args>
  bool dispatch(Args... params) const
  {
      if(m_dispatcher)
      {
          return m_dispatcher->on_event(params...);
      }
      return false;
  }

private:
  typename EventDispatcher::Ptr m_dispatcher;
};

}
}
