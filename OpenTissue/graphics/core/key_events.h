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

enum class KeyCode : uint16_t
{
  None                = 0,
  // ASCII
  Space               = 32,
  Apostrophe          = 39, /* ' */
  Comma               = 44, /* , */
  Minus               = 45, /* - */
  Period              = 46, /* . */
  Slash               = 47, /* / */

  D0                  = 48, /* 0 */
  D1                  = 49, /* 1 */
  D2                  = 50, /* 2 */
  D3                  = 51, /* 3 */
  D4                  = 52, /* 4 */
  D5                  = 53, /* 5 */
  D6                  = 54, /* 6 */
  D7                  = 55, /* 7 */
  D8                  = 56, /* 8 */
  D9                  = 57, /* 9 */

  Semicolon           = 59, /* ; */
  Equal               = 61, /* = */

  A                   = 65,
  B                   = 66,
  C                   = 67,
  D                   = 68,
  E                   = 69,
  F                   = 70,
  G                   = 71,
  H                   = 72,
  I                   = 73,
  J                   = 74,
  K                   = 75,
  L                   = 76,
  M                   = 77,
  N                   = 78,
  O                   = 79,
  P                   = 80,
  Q                   = 81,
  R                   = 82,
  S                   = 83,
  T                   = 84,
  U                   = 85,
  V                   = 86,
  W                   = 87,
  X                   = 88,
  Y                   = 89,
  Z                   = 90,

  LeftBracket         = 91,  /* [ */
  Backslash           = 92,  /* \ */
  RightBracket        = 93,  /* ] */
  GraveAccent         = 96,  /* ` */

  World1              = 161, /* non-US #1 */
  World2              = 162, /* non-US #2 */

  /* Function keys */
  Escape              = 256,
  Enter               = 257,
  Tab                 = 258,
  Backspace           = 259,
  Insert              = 260,
  Delete              = 261,
  Right               = 262,
  Left                = 263,
  Down                = 264,
  Up                  = 265,
  PageUp              = 266,
  PageDown            = 267,
  Home                = 268,
  End                 = 269,
  CapsLock            = 280,
  ScrollLock          = 281,
  NumLock             = 282,
  PrintScreen         = 283,
  Pause               = 284,
  F1                  = 290,
  F2                  = 291,
  F3                  = 292,
  F4                  = 293,
  F5                  = 294,
  F6                  = 295,
  F7                  = 296,
  F8                  = 297,
  F9                  = 298,
  F10                 = 299,
  F11                 = 300,
  F12                 = 301,
  F13                 = 302,
  F14                 = 303,
  F15                 = 304,
  F16                 = 305,
  F17                 = 306,
  F18                 = 307,
  F19                 = 308,
  F20                 = 309,
  F21                 = 310,
  F22                 = 311,
  F23                 = 312,
  F24                 = 313,
  F25                 = 314,

  /* Keypad */
  KP0                 = 320,
  KP1                 = 321,
  KP2                 = 322,
  KP3                 = 323,
  KP4                 = 324,
  KP5                 = 325,
  KP6                 = 326,
  KP7                 = 327,
  KP8                 = 328,
  KP9                 = 329,
  KPDecimal           = 330,
  KPDivide            = 331,
  KPMultiply          = 332,
  KPSubtract          = 333,
  KPAdd               = 334,
  KPEnter             = 335,
  KPEqual             = 336,

  LeftShift           = 340,
  LeftControl         = 341,
  LeftAlt             = 342,
  LeftSuper           = 343,
  RightShift          = 344,
  RightControl        = 345,
  RightAlt            = 346,
  RightSuper          = 347,
  Menu                = 348
};

class KeyEvent : public Event
{
public:
  static constexpr EventCategory Category = EventCategory::Keyboard;

  virtual ~KeyEvent() = default;

public:
  KeyCode get_code() const
  {
    return m_KeyCode;
  }

  EventCategory get_category_flag() const override
  {
    return EventCategory::Keyboard;
  }

protected:
  KeyEvent(const KeyCode keycode)
    : m_KeyCode(keycode) {}

  KeyCode m_KeyCode;
};

class KeyPressedEvent : public KeyEvent
{
public:
  static constexpr EventType Type = EventType::KeyPressed;
  static constexpr EventCategory Category = EventCategory::Keyboard;

  KeyPressedEvent(const KeyCode keycode, const uint16_t repeatCount = 0)
    : KeyEvent(keycode), m_RepeatCount(repeatCount) {}

  uint16_t repeat_count() const { return m_RepeatCount; }

  EventType get_event_type() const override
  {
    return Type;
  }

  const char* get_name() const override
  {
    return "KeyPressed";
  }

private:
  uint16_t m_RepeatCount;
};

class KeyReleasedEvent : public KeyEvent
{
public:
  static constexpr EventType Type = EventType::KeyReleased;
  static constexpr EventCategory Category = EventCategory::Keyboard;

  KeyReleasedEvent(const KeyCode keycode)
    : KeyEvent(keycode) {}

  EventType get_event_type() const override
  {
    return Type;
  }

  const char* get_name() const override
  {
    return "KeyReleased";
  }

};

class KeyTypedEvent : public KeyEvent
{
public:
  static constexpr EventType Type = EventType::KeyTyped;
  static constexpr EventCategory Category = EventCategory::Keyboard;

  KeyTypedEvent(const KeyCode keycode)
    : KeyEvent(keycode) {}

  EventType get_event_type() const override
  {
    return Type;
  }

  const char* get_name() const override
  {
    return "KeyTyped";
  }

};

// Conveniency operators for key modifiers opeartions.

inline int operator|(int a, KeyCode b)
{
  return a | static_cast<int>(b);
}

inline int operator|=(int a, KeyCode b)
{
  return a |= static_cast<int>(b);
}

inline int operator&(int a, KeyCode b)
{
  return a & static_cast<int>(b);
}

inline int operator&=(int a, KeyCode b)
{
  return a &= static_cast<int>(b);
}

inline bool operator==(int a, KeyCode b)
{
  return (a == static_cast<int>(b));
}

inline int operator|(KeyCode a, KeyCode b)
{
  return static_cast<int>(a) | b;
}

inline int operator|=(KeyCode a, KeyCode b)
{
  return static_cast<int>(a) |= b;
}

inline int operator&(KeyCode a, KeyCode b)
{
  return static_cast<int>(a) & b;
}

inline int operator&=(KeyCode a, KeyCode b)
{
  return static_cast<int>(a) &= b;
}

} // namespace graphics
} // namespace OpenTissue
