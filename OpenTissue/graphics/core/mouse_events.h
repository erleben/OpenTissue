//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include "OpenTissue/graphics/core/event.h"
#include "OpenTissue/graphics/core/key_events.h"

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
  static constexpr EventType Type = EventType::MouseMoved;
  static constexpr EventCategory Category = EventCategory::Mouse;

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

  EventType get_event_type() const override
  {
    return Type;
  }

  EventCategory get_category_flag() const override
  {
    return Category;
  }

  const char* get_name() const override
  {
    return "MouseMoved";
  }

private:
  MouseCode m_Button;
  bool m_passive;
  float m_mousex, m_mousey;
};

class MouseScrolledEvent : public Event
{
public:
  static constexpr EventType Type = EventType::MouseScrolled;
  static constexpr EventCategory Category = EventCategory::Mouse;

  MouseScrolledEvent(const float xoffset, const float yoffset, const int direction = 1)
    : m_xoffset(xoffset), m_yoffset(yoffset), m_direction(direction) {}

  inline float get_xoffset() const
  {
    return m_xoffset;
  }

  inline float get_yoffset() const
  {
    return m_yoffset;
  }

  inline float get_direction() const
  {
    return m_direction;
  }

  EventType get_event_type() const override
  {
    return Type;
  }

  EventCategory get_category_flag() const override
  {
    return Category;
  }

  const char* get_name() const override
  {
    return "MouseScrolled";
  }

private:
  float m_xoffset, m_yoffset, m_direction;
};

class MouseButtonEvent : public Event
{
public:
  static constexpr EventCategory Category = EventCategory::Mouse;

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

  inline KeyCode get_modifier() const
  {
    return m_modifier;
  }

  EventCategory get_category_flag() const override
  {
    return Category;
  }

protected:
  MouseButtonEvent(const MouseCode button, const float x, const float y)
    : m_Button(button), m_mousex(x), m_mousey(y) {}

  MouseButtonEvent(const MouseCode button, const KeyCode modifier, const float x, const float y)
    : m_Button(button), m_modifier(modifier), m_mousex(x), m_mousey(y) {}

  MouseCode m_Button;
  KeyCode   m_modifier;

  float m_mousex, m_mousey;
};

class MouseButtonPressedEvent : public MouseButtonEvent
{
public:
  static constexpr EventType Type = EventType::MouseButtonPressed;
  static constexpr EventCategory Category = EventCategory::Keyboard;

  MouseButtonPressedEvent(const MouseCode button, const float x = 0.0f, const float y = 0.0f)
    : MouseButtonEvent(button, x, y) {}

  MouseButtonPressedEvent(const MouseCode button, const KeyCode modifier, const float x, const float y)
    : MouseButtonEvent(button, modifier, x, y) {}

  EventType get_event_type() const override
  {
    return Type;
  }

  const char* get_name() const override
  {
    return "MouseButtonPressed";
  }
};

class MouseButtonReleasedEvent : public MouseButtonEvent
{
public:
  static constexpr EventType Type = EventType::MouseButtonReleased;
  static constexpr EventCategory Category = EventCategory::Keyboard;

  MouseButtonReleasedEvent(const MouseCode button, const float x = 0.0f, const float y = 0.0f)
    : MouseButtonEvent(button, x, y) {}

  EventType get_event_type() const override
  {
    return Type;
  }

  const char* get_name() const override
  {
    return "MouseButtonReleased";
  }
};

} // namespace graphics
} // namespace OpenTissue
