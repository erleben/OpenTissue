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

enum class MouseCode : uint16_t
{
  // From glfw3.h
  Button0                = 0,
  Button1                = 1,
  Button2                = 2,
  Button3                = 3,
  Button4                = 4,
  Button5                = 5,
  Button6                = 6,
  Button7                = 7,

  ButtonLast             = Button7,
  ButtonLeft             = Button0,
  ButtonRight            = Button1,
  ButtonMiddle           = Button2
};

class MouseMovedEvent : public Event
{
public:
  MouseMovedEvent(const float x, const float y)
    : m_mousex(x), m_mousey(y), m_passive(true) {}

  MouseMovedEvent(const MouseCode button, const float x, const float y)
    : m_Button(button), m_mousex(x), m_mousey(y), m_passive(false) {}

  float get_x() const 
  { 
    return m_mousex; 
  }

  float get_y() const 
  { 
    return m_mousey; 
  }

  bool is_passive() const 
  { 
    return m_passive; 
  }

  EventCategory get_category_flag() const
  {
    return EventCategory::Mouse;
  }

  EventType get_event_type() const
  {
    return EventType::MouseMoved;
  }

  const char* get_name()
  {
    return "MouseMoved";
  }

private:
  MouseCode m_Button;
  bool m_passive = false;
  float m_mousex, m_mousey;
};

class MouseScrolledEvent : public Event
{
public:
  MouseScrolledEvent(const float xoffset, const float yoffset)
    : m_xoffset(xoffset), m_yoffset(yoffset) {}

  inline float get_xoffset() const 
  { 
    return m_xoffset; 
  }

  inline float get_yoffset() const
  { 
    return m_yoffset; 
  }

  EventCategory get_category_flag() const
  {
    return EventCategory::Mouse;
  }

  EventType get_event_type() const
  {
    return EventType::MouseScrolled;
  }

  const char* get_name() const
  {
    return "MouseScrolled";
  }

private:
  float m_xoffset, m_yoffset;
};

class MouseButtonEvent : public Event
{
public:
  MouseCode get_mouse_button() const 
  { 
    return m_Button; 
  }

  inline float get_x() const 
  { 
    return m_mousex;
  }

  inline float get_y() const 
  { 
    return m_mousey; 
  }

  inline int get_modifier() const 
  { 
    return m_modifier; 
  }

  EventCategory get_category_flag() const
  {
    return EventCategory::Mouse;
  }

  virtual EventType get_event_type() const = 0;

  virtual const char* get_name() const = 0;

protected:
  MouseButtonEvent(const MouseCode button, float x, float y)
    : m_Button(button), m_mousex(x), m_mousey(y) {}

  MouseButtonEvent(const MouseCode button, const int modifier, float x, float y)
    : m_Button(button), m_modifier(modifier), m_mousex(x), m_mousey(y) {}

  MouseCode m_Button;
  int   m_modifier;

  float m_mousex, m_mousey;
};

class MouseButtonPressedEvent : public MouseButtonEvent
{
public:
  MouseButtonPressedEvent(const MouseCode button, float x, float y)
    : MouseButtonEvent(button, x, y) {}

  MouseButtonPressedEvent(const MouseCode button, const int modifier, float x, float y)
    : MouseButtonEvent(button, modifier, x, y) {}

  EventType get_event_type() const
  {
    return EventType::MouseButtonPressed;
  }

  const char* get_name() const
  {
    return "MouseButtonPressed";
  }
};

class MouseButtonReleasedEvent : public MouseButtonEvent
{
public:
  MouseButtonReleasedEvent(const MouseCode button, float x, float y)
    : MouseButtonEvent(button, x, y) {}

  EventType get_event_type() const
  {
    return EventType::MouseButtonReleased;
  }

  const char* get_name() const
  {
    return "MouseButtonReleased";
  }
};

}
}
