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

//= INCLUDES ===============
#include "../Resource/IResource.h"
#include "../Math/Vector3.h"
#include "../Math/Quaternion.h"
#include "../Math/Matrix.h"
//==========================

namespace spartan
{
    // Animation keyframe (template supports Vector3 and Quaternion)
    template<typename T>
    struct AnimationKey
    {
        float time;
        T value;
    };

    // Single bone's animation track
    struct AnimationChannel
    {
        std::string bone_name;
        std::vector<AnimationKey<math::Vector3>> position_keys;
        std::vector<AnimationKey<math::Quaternion>> rotation_keys;
        std::vector<AnimationKey<math::Vector3>> scale_keys;
    };

    class Animation : public IResource
    {
    public:
        Animation() : IResource(ResourceType::Animation) {}
        ~Animation() = default;

        // IResource
        void LoadFromFile(const std::string& file_path) override;
        void SaveToFile(const std::string& file_path) override;

        // Animation data
        void AddChannel(const std::string& bone_name,
                        const std::vector<AnimationKey<math::Vector3>>& position_keys,
                        const std::vector<AnimationKey<math::Quaternion>>& rotation_keys,
                        const std::vector<AnimationKey<math::Vector3>>& scale_keys);

        // Animation playback
        math::Matrix SampleBone(const std::string& bone_name, float time_ticks) const;

        // Properties
        void SetDurationInSeconds(float duration) { m_duration_in_seconds = duration; }
        float GetDurationInSeconds() const { return m_duration_in_seconds; }
        void SetTicksPerSecond(double ticks) { m_ticks_per_second = ticks; }
        double GetTicksPerSecond() const { return m_ticks_per_second; }
        const std::vector<AnimationChannel>& GetChannels() const { return m_channels; }

    private:
        // Interpolation helpers
        math::Vector3 LerpV3(const std::vector<AnimationKey<math::Vector3>>& keys, float t) const;
        math::Quaternion SlerpQ(const std::vector<AnimationKey<math::Quaternion>>& keys, float t) const;

        // Animation data
        std::vector<AnimationChannel> m_channels;
        float m_duration_in_seconds = 0.0f;
        double m_ticks_per_second = 0.0;
    };
}
