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

//= INCLUDES =========
#include "../../Math/Matrix.h"
#include "../../Memory/Allocator.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
//====================

namespace spartan
{
    struct Skeleton
    {
        void ComputeGlobalPose(std::span<const math::Matrix> local_matrices, std::span<math::Matrix> out_global_matrices) const
        {
            const std::size_t bone_count = parent_indices.size();
            if (local_matrices.size() < bone_count || out_global_matrices.size() < bone_count)
                return;

            for (std::size_t bone_index = 0; bone_index < bone_count; ++bone_index)
            {
                const int16_t parent_index = parent_indices[bone_index];
                out_global_matrices[bone_index] = parent_index < 0
                    ? local_matrices[bone_index]
                    : local_matrices[bone_index] * out_global_matrices[static_cast<std::size_t>(parent_index)];
            }
        }

        void Allocate(const uint16_t in_joint_count)
        {
            Clear();
            joint_count = in_joint_count;
            if (joint_count == 0)
                return;

            std::array<std::size_t, 5> offsets = {};
            std::size_t total_bytes = 0;

            offsets[0] = total_bytes = AlignUp<int16_t>(total_bytes);
            total_bytes += static_cast<std::size_t>(joint_count) * sizeof(int16_t);

            offsets[1] = total_bytes = AlignUp<math::Vector3>(total_bytes);
            total_bytes += static_cast<std::size_t>(joint_count) * sizeof(math::Vector3);

            offsets[2] = total_bytes = AlignUp<math::Quaternion>(total_bytes);
            total_bytes += static_cast<std::size_t>(joint_count) * sizeof(math::Quaternion);

            offsets[3] = total_bytes = AlignUp<math::Vector3>(total_bytes);
            total_bytes += static_cast<std::size_t>(joint_count) * sizeof(math::Vector3);

            offsets[4] = total_bytes = AlignUp<math::Matrix>(total_bytes);
            total_bytes += static_cast<std::size_t>(joint_count) * sizeof(math::Matrix);

            constexpr std::size_t storage_alignment = std::max({ alignof(int16_t), alignof(math::Vector3), alignof(math::Quaternion), alignof(math::Matrix) });
            m_storage.reset(static_cast<std::byte*>(Allocator::Allocate(total_bytes, storage_alignment)));

            std::byte* storage = m_storage.get();
            AssignSpan(storage, offsets[0], parent_indices, joint_count);
            AssignSpan(storage, offsets[1], bind_positions, joint_count);
            AssignSpan(storage, offsets[2], bind_rotations, joint_count);
            AssignSpan(storage, offsets[3], bind_scales, joint_count);
            AssignSpan(storage, offsets[4], inverse_bind_matrices, joint_count);
        }

        // Bones are stored in topological order so parents always come before children.
        uint16_t joint_count = 0;

        std::span<int16_t> parent_indices;
        std::span<math::Vector3> bind_positions;
        std::span<math::Quaternion> bind_rotations;
        std::span<math::Vector3> bind_scales;
        std::span<math::Matrix> inverse_bind_matrices;

    private:
        struct StorageDeleter
        {
            void operator()(std::byte* ptr) const
            {
                Allocator::Free(ptr);
            }
        };

        friend class SkeletonReader;

        void Clear()
        {
            m_storage.reset();
            joint_count = 0;
            parent_indices = std::span<int16_t>{};
            bind_positions = std::span<math::Vector3>{};
            bind_rotations = std::span<math::Quaternion>{};
            bind_scales = std::span<math::Vector3>{};
            inverse_bind_matrices = std::span<math::Matrix>{};
        }

        template <typename T>
        static std::size_t AlignUp(const std::size_t offset)
        {
            const std::size_t alignment = alignof(T);
            const std::size_t remainder = offset % alignment;
            return remainder == 0 ? offset : offset + (alignment - remainder);
        }

        template <typename T>
        static void AssignSpan(std::byte* storage, const std::size_t offset, std::span<T>& out_span, const uint16_t count)
        {
            out_span = std::span<T>(reinterpret_cast<T*>(storage + offset), count);
        }

        std::unique_ptr<std::byte, StorageDeleter> m_storage;
    };
}
