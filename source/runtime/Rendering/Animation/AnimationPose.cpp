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

//= INCLUDES ==========================
#include "../../Core/pch.h"
#include "AnimationPose.h"
//=====================================

namespace spartan
{
    void AnimationPose::Resize(const uint32_t bone_count)
    {
        local_positions.resize(bone_count, math::Vector3::Zero);
        local_rotations.resize(bone_count, math::Quaternion::Identity);
        local_scales.resize(bone_count, math::Vector3::One);
        global_matrices.resize(bone_count, math::Matrix::Identity);
        skinning_matrices.resize(bone_count, math::Matrix::Identity);
    }

    void AnimationPose::ResetToBindPose(const Skeleton& skeleton)
    {
        const uint32_t bone_count = static_cast<uint32_t>(skeleton.parent_indices.size());
        Resize(bone_count);

        local_positions.assign(skeleton.bind_positions.begin(), skeleton.bind_positions.end());
        local_rotations.assign(skeleton.bind_rotations.begin(), skeleton.bind_rotations.end());
        local_scales.assign(skeleton.bind_scales.begin(), skeleton.bind_scales.end());

        if (bone_count > 0)
        {
            std::vector<math::Matrix> local_matrices(bone_count);
            for (uint32_t bone_index = 0; bone_index < bone_count; ++bone_index)
            {
                local_matrices[bone_index] = math::Matrix(local_positions[bone_index], local_rotations[bone_index], local_scales[bone_index]);
            }

            skeleton.ComputeGlobalPose(local_matrices, global_matrices);
        }

        if (skeleton.inverse_bind_matrices.size() == bone_count)
        {
            for (uint32_t i = 0; i < bone_count; i++)
            {
                skinning_matrices[i] = skeleton.inverse_bind_matrices[i] * global_matrices[i];
            }
        }
    }
}
