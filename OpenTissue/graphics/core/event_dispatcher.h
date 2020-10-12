//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <functional>
#include <unordered_map>
#include <vector>

#include "OpenTissue/graphics/core/event.h"
#include "OpenTissue/graphics/core/application_events.h"
#include "OpenTissue/graphics/core/key_events.h"
#include "OpenTissue/graphics/core/mouse_events.h"

namespace OpenTissue {
namespace graphics {

/**
 * @brief Event dispatch structure.
 *  Subscribe listeners to this class.
 *
 */
class EventDispatcher
{
public:
  using CallbackFnType = std::function<void(const Event&)>;

  void post(const Event& e) const
  {
    auto type = e.get_event_type();

    // Ignore events for which we do not have an observer (yet).
    if(m_observers.find(type) == m_observers.end())
    {
      return;
    }

    auto &&observers = m_observers.at(type);
    for(auto &&fn : observers)
    {
      if(!e.handled())
      {
        fn(e);
      }
    }
  }

  template<typename ListenerType>
  void subscribeListener(ListenerType *listener)
  {
    this->subscribe(EventType::WindowClose,
    [listener](const Event &e)
    {
      listener->on_window_close(static_cast<const WindowCloseEvent&>(e));
    });
    this->subscribe(EventType::WindowResize,
    [listener](const Event &e)
    {
      listener->on_window_resize(static_cast<const WindowResizeEvent&>(e));
    });
    this->subscribe(EventType::WindowDisplay,
    [listener](const Event &e)
    {
      listener->on_window_display(static_cast<const WindowDisplayEvent&>(e));
    });
    this->subscribe(EventType::KeyPressed,
    [listener](const Event &e)
    {
      listener->on_key_pressed(static_cast<const KeyPressedEvent&>(e));
    });
    this->subscribe(EventType::MouseButtonPressed,
    [listener](const Event &e)
    {
      listener->on_mouse_button_pressed(static_cast<const MouseButtonPressedEvent&>(e));
    });
    this->subscribe(EventType::MouseButtonReleased,
    [listener](const Event &e)
    {
      listener->on_mouse_button_released(static_cast<const MouseButtonReleasedEvent&>(e));
    });
    this->subscribe(EventType::MouseMoved,
    [listener](const Event &e)
    {
      listener->on_mouse_moved(static_cast<const MouseMovedEvent&>(e));
    });
    this->subscribe(EventType::MouseScrolled,
    [listener](const Event &e)
    {
      listener->on_mouse_scrolled(static_cast<const MouseScrolledEvent&>(e));
    });
  }

private:
  void subscribe(const EventType& type, CallbackFnType&& fn)
  {
    m_observers[type].push_back(fn);
  }

private:
 std::unordered_map<EventType, std::vector<CallbackFnType>> m_observers;
};

} // namespace graphics
} // namespace OpenTissue
