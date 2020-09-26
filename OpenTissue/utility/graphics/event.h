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

class Event
{
public:
  using Self = Event;
  using Ptr  = std::shared_ptr<Self>;

public:
  Event() = default;
  virtual ~Event() = default;

  inline bool handled() const
  {
    return m_handled;
  }

  virtual EventType get_event_type() const
  {
    return EventType::None;
  }

  virtual EventCategory get_category_flag() const
  {
    return EventCategory::None;
  }

  virtual const char* get_name() const
  {
    return "None";
  }

protected:
  bool m_handled = false;
};

}
}
