//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include "OpenTissue/utility/graphics/event.h"

namespace OpenTissue {
namespace graphics {
  
class WindowResizeEvent : public Event
{
public:
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
    return EventType::WindowResize;
  }

  EventCategory get_category_flag() const
  {
    return EventCategory::Application;
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
  WindowCloseEvent() = default;

  EventType get_event_type() const
  {
    return EventType::WindowClose;
  }

  EventCategory get_category_flag() const
  {
    return EventCategory::Application;
  }

  const char* get_name() const
  {
    return "WindowClose";
  }
};

class WindowDisplayEvent : public Event
{
public:
  WindowDisplayEvent() = default;

  EventType get_event_type() const
  {
    return EventType::WindowDisplay;
  }

  EventCategory get_category_flag() const
  {
    return EventCategory::Application;
  }

  const char* get_name() const
  {
    return "WindowDisplayEvent";
  }
};

}
}
