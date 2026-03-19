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
#include "AnimationSamplingCommon.h"
//===============================

namespace spartan
{
    void AnimationClip::SampleToMatrices(const float time_seconds, std::span<math::Matrix> out) const
    {
        SP_ASSERT(out.size() >= joint_count);
        SP_ASSERT(base_local_positions.size() == joint_count);
        SP_ASSERT(base_local_rotations.size() == joint_count);
        SP_ASSERT(base_local_scales.size() == joint_count);

        if (joint_count == 0)
            return;

        const float clamped_time = std::clamp(time_seconds, 0.0f, duration_seconds);
        const float sample_position = std::max(clamped_time * sample_rate, 0.0f);
        const uint32_t sample_index = static_cast<uint32_t>(sample_position);
        const float alpha = sample_position - static_cast<float>(sample_index);

        thread_local std::vector<math::Vector3> sampled_positions;
        thread_local std::vector<math::Quaternion> sampled_rotations;
        thread_local std::vector<math::Vector3> sampled_scales;

        sampled_positions.assign(base_local_positions.begin(), base_local_positions.end());
        sampled_rotations.assign(base_local_rotations.begin(), base_local_rotations.end());
        sampled_scales.assign(base_local_scales.begin(), base_local_scales.end());

        animation_runtime::ApplyConstantChannels(position_stream.constants, sampled_positions, joint_count);
        animation_runtime::ApplyConstantChannels(rotation_stream.constants, sampled_rotations, joint_count);
        animation_runtime::ApplyConstantChannels(scale_stream.constants, sampled_scales, joint_count);

        animation_runtime::ApplySampledChannels(
            position_stream.channels, position_stream.values, sampled_positions, joint_count, sample_index, alpha,
            [](const math::Vector3& a, const math::Vector3& b, const float t) { return math::Vector3::Lerp(a, b, t); });

        animation_runtime::ApplySampledChannels(
            rotation_stream.channels, rotation_stream.values, sampled_rotations, joint_count, sample_index, alpha,
            [](const math::Quaternion& a, const math::Quaternion& b, const float t) { return math::Quaternion::Slerp(a, b, t); });

        animation_runtime::ApplySampledChannels(
            scale_stream.channels, scale_stream.values, sampled_scales, joint_count, sample_index, alpha,
            [](const math::Vector3& a, const math::Vector3& b, const float t) { return math::Vector3::Lerp(a, b, t); });

        for (uint32_t i = 0; i < joint_count; ++i)
        {
            out[i] = math::Matrix(sampled_positions[i], sampled_rotations[i], sampled_scales[i]);
        }
    }
}
