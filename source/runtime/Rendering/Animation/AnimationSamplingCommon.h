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

#include "AnimationClip.h"
#include <span>

namespace spartan::animation_runtime
{
    template <typename TValue, typename TInterpolator>
    [[nodiscard]] inline TValue sample_track_value(
        const std::span<const TValue> values,
        const spartan::AnimChannel& channel,
        const uint32_t sample_index,
        const float alpha,
        const TValue& fallback,
        TInterpolator&& interpolator
    )
    {
        const uint32_t first = channel.first_sample;
        const uint32_t count = channel.sample_count;

        if (count == 0)
            return fallback;

        if (count == 1)
            return first < values.size() ? values[first] : fallback;

        if ((first + count) > values.size())
            return fallback;

        const uint32_t clamped_index = std::min(sample_index, count - 1u);
        if (clamped_index >= count - 1u)
            return values[first + count - 1u];

        const uint32_t i0 = first + clamped_index;
        return interpolator(values[i0], values[i0 + 1u], alpha);
    }

    template <typename TChannel, typename TValue>
    inline void ApplyConstantChannels(
        const std::vector<TChannel>& channels,
        std::vector<TValue>& output,
        const uint32_t bone_count)
    {
        for (const TChannel& channel : channels)
        {
            SP_ASSERT(channel.bone_index < bone_count);
            if (channel.bone_index < bone_count)
            {
                output[channel.bone_index] = channel.value;
            }
        }
    }

    template <typename TChannel, typename TValue, typename TInterpolator>
    inline void ApplySampledChannels(
        const std::vector<TChannel>& channels,
        const std::vector<TValue>& values,
        std::vector<TValue>& output,
        const uint32_t bone_count,
        const uint32_t sample_index,
        const float alpha,
        TInterpolator&& interpolator)
    {
        const std::span<const TValue> value_span(values.data(), values.size());

        for (const TChannel& channel : channels)
        {
            SP_ASSERT(channel.bone_index < bone_count);
            output[channel.bone_index] = sample_track_value(
                value_span, channel, sample_index, alpha, output[channel.bone_index],
                std::forward<TInterpolator>(interpolator)
            );
        }
    }
}
