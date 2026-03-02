/*
Copyright(c) 2015-2026 Panos Karabelas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "pch.h"
#include "SkinningGeometryBuffer.h"
#include "../RHI/RHI_Buffer.h"
#include "../RHI/RHI_Device.h"

namespace spartan
{
    // Static member definitions
    std::vector<RHI_Vertex_PosTexNorTan> SkinningGeometryBuffer::m_cpu_vertices;
    std::vector<uint32_t>                SkinningGeometryBuffer::m_cpu_indices;
    std::vector<float>                   SkinningGeometryBuffer::m_cpu_weights;
    uint32_t                             SkinningGeometryBuffer::m_vertex_cursor = 0;
    std::mutex                           SkinningGeometryBuffer::m_mutex;
    std::shared_ptr<RHI_Buffer>          SkinningGeometryBuffer::m_buf_vertices;
    std::shared_ptr<RHI_Buffer>          SkinningGeometryBuffer::m_buf_indices;
    std::shared_ptr<RHI_Buffer>          SkinningGeometryBuffer::m_buf_weights;

    void SkinningGeometryBuffer::Initialize()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cpu_vertices.reserve(k_max_vertices);
        m_cpu_indices.reserve(k_max_vertices);   // 1 uint32 per vertex (4 x uint8 packed)
        m_cpu_weights.reserve(k_max_vertices * 4);
        m_vertex_cursor = 0;
    }

    void SkinningGeometryBuffer::Shutdown()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cpu_vertices.clear();
        m_cpu_indices.clear();
        m_cpu_weights.clear();
        m_buf_vertices.reset();
        m_buf_indices.reset();
        m_buf_weights.reset();
    }

    uint32_t SkinningGeometryBuffer::AppendVertices(const std::vector<RHI_Vertex_PosTexNorTan>& vertices)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        SP_ASSERT(m_vertex_cursor + vertices.size() <= k_max_vertices);
        const uint32_t offset = m_vertex_cursor;
        m_cpu_vertices.insert(m_cpu_vertices.end(), vertices.begin(), vertices.end());
        m_vertex_cursor += static_cast<uint32_t>(vertices.size());
        return offset;
    }

    uint32_t SkinningGeometryBuffer::AppendBoneIndices(const std::vector<uint8_t>& indices)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // 'indices' is 4*N bytes; pack every 4 bytes into one uint32
        SP_ASSERT(indices.size() % 4 == 0);
        const uint32_t offset = static_cast<uint32_t>(m_cpu_indices.size());
        const uint32_t vertex_count = static_cast<uint32_t>(indices.size()) / 4;
        m_cpu_indices.reserve(m_cpu_indices.size() + vertex_count);
        for (uint32_t i = 0; i < vertex_count; i++)
        {
            uint32_t packed = 0;
            packed |= (uint32_t)indices[i * 4 + 0] << 0;
            packed |= (uint32_t)indices[i * 4 + 1] << 8;
            packed |= (uint32_t)indices[i * 4 + 2] << 16;
            packed |= (uint32_t)indices[i * 4 + 3] << 24;
            m_cpu_indices.push_back(packed);
        }
        return offset;
    }

    uint32_t SkinningGeometryBuffer::AppendBoneWeights(const std::vector<float>& weights)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        SP_ASSERT(weights.size() % 4 == 0);
        const uint32_t offset = static_cast<uint32_t>(m_cpu_weights.size()) / 4; // element index = vertex index
        m_cpu_weights.insert(m_cpu_weights.end(), weights.begin(), weights.end());
        return offset;
    }

    void SkinningGeometryBuffer::Flush()
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_cpu_vertices.empty())
            return;

        SP_ASSERT(m_cpu_indices.size() == m_cpu_vertices.size());
        SP_ASSERT((m_cpu_weights.size() % 4) == 0);
        SP_ASSERT((m_cpu_weights.size() / 4) == m_cpu_vertices.size());

        // Move old buffers to deletion queue to ensure GPU is done using them
        if (m_buf_vertices)
        {
            RHI_Device::DeletionQueueAdd(RHI_Resource_Type::Buffer, m_buf_vertices->GetRhiResource());
            m_buf_vertices.reset();
        }
        if (m_buf_indices)
        {
            RHI_Device::DeletionQueueAdd(RHI_Resource_Type::Buffer, m_buf_indices->GetRhiResource());
            m_buf_indices.reset();
        }
        if (m_buf_weights)
        {
            RHI_Device::DeletionQueueAdd(RHI_Resource_Type::Buffer, m_buf_weights->GetRhiResource());
            m_buf_weights.reset();
        }

        m_buf_vertices = std::make_shared<RHI_Buffer>(
            RHI_Buffer_Type::Storage,
            static_cast<uint32_t>(sizeof(RHI_Vertex_PosTexNorTan)),
            static_cast<uint32_t>(m_cpu_vertices.size()),
            m_cpu_vertices.data(),
            true,
            "skinning_vertices_in"
        );

        m_buf_indices = std::make_shared<RHI_Buffer>(
            RHI_Buffer_Type::Storage,
            static_cast<uint32_t>(sizeof(uint32_t)),
            static_cast<uint32_t>(m_cpu_indices.size()),
            m_cpu_indices.data(),
            true,
            "skinning_bone_indices"
        );

        m_buf_weights = std::make_shared<RHI_Buffer>(
            RHI_Buffer_Type::Storage,
            static_cast<uint32_t>(sizeof(float) * 4),
            static_cast<uint32_t>(m_cpu_weights.size() / 4),
            m_cpu_weights.data(),
            true,
            "skinning_bone_weights"
        );

        // Keep CPU staging memory so subsequent Flush() calls can rebuild buffers
        // with all previously appended data (offsets remain stable across imports).
    }
}
