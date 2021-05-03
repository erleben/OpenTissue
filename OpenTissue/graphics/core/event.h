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
  constexpr Event() = default;

  virtual EventType get_event_type() const = 0;

  virtual EventCategory get_category_flag() const = 0;

  virtual const char* get_name() const = 0;

  bool handled() const
  {
    return m_handled;
  }

  bool handled(bool is_handled)
  {
    m_handled = is_handled;
  }

protected:
  bool m_handled = false;
};

// Observer pattern based dispatcher.

} // namespace graphics
} // namespace OpenTissue
