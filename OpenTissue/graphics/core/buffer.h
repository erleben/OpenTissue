//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

#include <OpenTissue/graphics/core/gl/gl_util.h>

#include <string>
#include <vector>
#include <memory>

namespace OpenTissue {
namespace graphics {

enum class ShaderDataType
{
  None = 0, Float, Float2, Float3, Float4, Mat3, Mat4, Int, Int2, Int3, Int4, Bool
};

static uint32_t shader_data_type_size(ShaderDataType type)
{
  switch(type)
  {
    case ShaderDataType::Float:    return 4;
    case ShaderDataType::Float2:   return 4 * 2;
    case ShaderDataType::Float3:   return 4 * 3;
    case ShaderDataType::Float4:   return 4 * 4;
    case ShaderDataType::Mat3:     return 4 * 3 * 3;
    case ShaderDataType::Mat4:     return 4 * 4 * 4;
    case ShaderDataType::Int:      return 4;
    case ShaderDataType::Int2:     return 4 * 2;
    case ShaderDataType::Int3:     return 4 * 3;
    case ShaderDataType::Int4:     return 4 * 4;
    case ShaderDataType::Bool:     return 1;
  }

  return 0;
}

class BufferElement
{
public:
  BufferElement() = default;

  BufferElement(ShaderDataType type, const std::string& name, bool normalized = false)
    : m_name(name), m_type(type), m_size(ShaderDataTypeSize(type)), m_offset(0), m_normalized(normalized)
  {
  }

  uint32_t get_component_count() const
  {
    switch (Type)
    {
      case ShaderDataType::Float:   return 1;
      case ShaderDataType::Float2:  return 2;
      case ShaderDataType::Float3:  return 3;
      case ShaderDataType::Float4:  return 4;
      case ShaderDataType::Mat3:    return 3; // 3* float3
      case ShaderDataType::Mat4:    return 4; // 4* float4
      case ShaderDataType::Int:     return 1;
      case ShaderDataType::Int2:    return 2;
      case ShaderDataType::Int3:    return 3;
      case ShaderDataType::Int4:    return 4;
      case ShaderDataType::Bool:    return 1;
    }

    return 0;
  }

public:
  std::string m_name;
  ShaderDataType m_type;
  uint32_t m_size;
  size_t m_offset;
  bool m_normalized;
};

class BufferLayout
{
public:
  BufferLayout() = default;

  BufferLayout(const std::initializer_list<BufferElement> &elements)
    : m_elements(elements)
  {
    init_offset_and_stride();
  }

  uint32_t get_stride() const { return m_stride; }
  const std::vector<BufferElement>& get_elements() const { return m_elements; }

  std::vector<BufferElement>::iterator begin() { return m_elements.begin(); }
  std::vector<BufferElement>::iterator end() { return m_elements.end(); }
  std::vector<BufferElement>::const_iterator begin() const { return m_elements.begin(); }
  std::vector<BufferElement>::const_iterator end() const { return m_elements.end(); }

private:
  void init_offset_and_stride()
  {
    size_t offset = 0;
    m_stride = 0;
    for (auto& element : m_elements)
    {
      element.m_offset = offset;
      offset += element.m_size;
      m_stride += element.m_size;
    }
  }

private:
  std::vector<BufferElement> m_elements;
  uint32_t m_stride = 0;
};

template<typename BufferType>
class Buffer
{
public:
  using Self = Buffer<BufferType>;
  using Ptr  = std::shared_ptr<Self>;

  template<typename... Args>
  static Ptr New(Args&&... args)
  {
    return std::static_pointer_cast<Self>(
        std::make_shared<BufferType>(args...));
  }

public:
  Buffer() = delete;
  Buffer(uint32_t* indices, uint32_t count) : m_count(count)
  {
    glCreateBuffers(1, &m_rendererid);

		// GL_ELEMENT_ARRAY_BUFFER is not valid without an actively bound VAO
		// Binding with GL_ARRAY_BUFFER allows the data to be loaded regardless of VAO state.
		glBindBuffer(GL_ARRAY_BUFFER, m_rendererid);
		glBufferData(GL_ARRAY_BUFFER, count * sizeof(uint32_t), indices, GL_STATIC_DRAW);
  }

  Buffer(uint32_t size)
  {
    glCreateBuffers(1, &m_rendererid);
		glBindBuffer(GL_ARRAY_BUFFER, m_rendererid);
		glBufferData(GL_ARRAY_BUFFER, size, nullptr, GL_DYNAMIC_DRAW);
  }

  Buffer(float* vertices, uint32_t size)
  {
		glCreateBuffers(1, &m_rendererid);
		glBindBuffer(GL_ARRAY_BUFFER, m_rendererid);
		glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);
  }

  virtual ~Buffer()
  {
		glDeleteBuffers(1, &m_rendererid);
  }

  void bind() const
  {
		static_cast<BufferType*>(this)->bind();
  }

  void unbind() const
  {
		static_cast<BufferType*>(this)->unbind();
  }

  void set_data(const void* data, uint32_t size)
  {
		static_cast<BufferType*>(this)->set_data(data, size);
  }

  const BufferLayout& get_layout() const
  {
    return m_Layout;
  }

  void set_layout(const BufferLayout& layout)
  {
    m_Layout = layout;
  }

  uint32_t get_count() const
  {
    return m_count;
  }

private:
  BufferLayout m_Layout;
  uint32_t m_rendererid;
  uint32_t m_count = 0;
};

class VertexBuffer : public Buffer<VertexBuffer>
{
public:
  template<typename... Args>
  VertexBuffer(Args... args) : Buffer<VertexBuffer>(args) {}

  void bind() const
  {
		glBindBuffer(GL_ARRAY_BUFFER, m_rendererid);
  }

  void unbind() const
  {
		glBindBuffer(GL_ARRAY_BUFFER, 0);
  };

  void set_data(const void* data, uint32_t size)
  {
		glBindBuffer(GL_ARRAY_BUFFER, m_rendererid);
		glBufferSubData(GL_ARRAY_BUFFER, 0, size, data);
  }
};

class IndexBuffer : public Buffer<IndexBuffer>
{
public:
  template<typename... Args>
  IndexBuffer(Args... args) : Buffer<IndexBuffer>(args) {}

  void bind() const
  {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_rendererid);
  }

  void unbind() const
  {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  void set_data(const void* data, uint32_t size)
  {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_rendererid);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, size, data);
  }

};

}
}
