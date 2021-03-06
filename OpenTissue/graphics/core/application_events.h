//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include "OpenTissue/graphics/core/event.h"

namespace OpenTissue {
namespace graphics {

class WindowResizeEvent : public Event
{
public:
  static constexpr EventType Type = EventType::WindowResize;
  static constexpr EventCategory Category = EventCategory::Application;

  WindowResizeEvent(unsigned int width, unsigned int height)
    : m_width(width), m_height(height) {}

  unsigned int get_width() const
  {
    return m_width;
  }

  unsigned int get_height() const
  {
    return m_height;
  }

  EventType get_event_type() const
  {
    return Type;
  }

  EventCategory get_category_flag() const
  {
    return Category;
  }

  const char* get_name() const
  {
    return "WindowResize";
  }

private:
  unsigned int m_width, m_height;
};

class WindowCloseEvent : public Event
{
public:
  static constexpr EventType Type = EventType::WindowClose;
  static constexpr EventCategory Category = EventCategory::Application;

  WindowCloseEvent() = default;

  EventType get_event_type() const
  {
    return Type;
  }

  EventCategory get_category_flag() const
  {
    return Category;
  }

  const char* get_name() const
  {
    return "WindowClose";
  }
};

class WindowDisplayEvent : public Event
{
public:
  static constexpr EventType Type = EventType::WindowDisplay;
  static constexpr EventCategory Category = EventCategory::Application;

  WindowDisplayEvent() = default;

  EventType get_event_type() const
  {
    return Type;
  }

  EventCategory get_category_flag() const
  {
    return Category;
  }

  const char* get_name() const
  {
    return "WindowDisplayEvent";
  }
};

} // namespace graphics
} // namespace OpenTissue
