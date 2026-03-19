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

//= INCLUDES ====================
#include "pch.h"
#include "AnimationClip.h"
//===============================

namespace spartan
{
    void AnimationClip::SampleToMatrices(const float time_seconds, std::span<math::Matrix> out) const
    {
        SP_ASSERT(out.size() >= joint_count);
        SP_ASSERT(base_local_positions.size() == joint_count);
        SP_ASSERT(base_local_rotations.size() == joint_count);
        SP_ASSERT(base_local_scales.size() == joint_count);

        thread_local std::vector<math::Vector3> pos;
        thread_local std::vector<math::Quaternion> rot;
        thread_local std::vector<math::Vector3> scl;

        pos.resize(joint_count);
        rot.resize(joint_count);
        scl.resize(joint_count);

        std::copy(base_local_positions.begin(), base_local_positions.end(), pos.begin());
        std::copy(base_local_rotations.begin(), base_local_rotations.end(), rot.begin());
        std::copy(base_local_scales.begin(), base_local_scales.end(), scl.begin());

        if (sample_count >= 2)
        {
            const float clamped_time = std::clamp(time_seconds, 0.0f, duration_seconds);
            const float f            = clamped_time * sample_rate;
            const uint32_t idx       = std::min(static_cast<uint32_t>(f), sample_count - 2);
            const float alpha        = f - static_cast<float>(idx);

            for (const auto& c : scale_stream.channels)
            {
                scl[c.bone_index] = math::Vector3::Lerp(
                    scale_stream.values[c.first_sample + idx],
                    scale_stream.values[c.first_sample + idx + 1],
                    alpha
                );
            }

            for (const auto& c : rotation_stream.channels)
            {
                rot[c.bone_index] = math::Quaternion::Slerp(
                    rotation_stream.values[c.first_sample + idx],
                    rotation_stream.values[c.first_sample + idx + 1],
                    alpha
                );
            }

            for (const auto& c : position_stream.channels)
            {
                pos[c.bone_index] = math::Vector3::Lerp(
                    position_stream.values[c.first_sample + idx],
                    position_stream.values[c.first_sample + idx + 1],
                    alpha
                );
            }
        }

        for (const auto& c : scale_stream.constants)    scl[c.bone_index] = c.value;
        for (const auto& c : rotation_stream.constants) rot[c.bone_index] = c.value;
        for (const auto& c : position_stream.constants) pos[c.bone_index] = c.value;

        for (uint32_t i = 0; i < joint_count; ++i)
        {
            out[i] = math::Matrix(pos[i], rot[i], scl[i]);
        }
    }
}
