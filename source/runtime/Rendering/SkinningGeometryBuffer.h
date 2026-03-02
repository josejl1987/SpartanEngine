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

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include "../RHI/RHI_Buffer.h"
#include "../RHI/RHI_Vertex.h"

namespace spartan
{
    // Linear append-only GPU buffer manager for static skinning input data.
    // All skinned meshes write once at load time; offset is permanent for the session.
    class SkinningGeometryBuffer
    {
    public:
        static constexpr uint32_t k_max_vertices = 1024 * 1024; // 1M vertices

        static void Initialize();
        static void Shutdown();

        // Append vertex data for a skinned mesh. Returns the global element offset
        // (i.e. the index of the first vertex in the global input buffer).
        static uint32_t AppendVertices(const std::vector<RHI_Vertex_PosTexNorTan>& vertices);

        // Append packed bone indices (4 x uint8 per vertex, stored as uint32).
        // 'indices' must have exactly vertices.size() * 4 entries.
        static uint32_t AppendBoneIndices(const std::vector<uint8_t>& indices);

        // Append bone weights (4 x float per vertex).
        // 'weights' must have exactly vertices.size() * 4 entries.
        static uint32_t AppendBoneWeights(const std::vector<float>& weights);

        // Flush all pending CPU-side data to GPU (call once after all meshes are loaded).
        static void Flush();

        static RHI_Buffer* GetVerticesBuffer()     { return m_buf_vertices.get(); }
        static RHI_Buffer* GetIndicesBuffer()      { return m_buf_indices.get(); }
        static RHI_Buffer* GetWeightsBuffer()      { return m_buf_weights.get(); }
        static uint32_t    GetVertexCount()        { return m_vertex_cursor; }

    private:
        // CPU staging (written at load time)
        static std::vector<RHI_Vertex_PosTexNorTan> m_cpu_vertices;
        static std::vector<uint32_t>                m_cpu_indices;  // 4 x uint8 packed into uint32
        static std::vector<float>                   m_cpu_weights;
        static uint32_t                             m_vertex_cursor;
        static std::mutex                           m_mutex;

        // GPU buffers (Storage, immutable after Flush())
        static std::shared_ptr<RHI_Buffer>          m_buf_vertices;
        static std::shared_ptr<RHI_Buffer>          m_buf_indices;
        static std::shared_ptr<RHI_Buffer>          m_buf_weights;
    };
}
