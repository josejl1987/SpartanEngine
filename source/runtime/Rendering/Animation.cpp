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

//= INCLUDES ===============
#include "pch.h"
#include "Animation.h"
#include "../Resource/ResourceCache.h"
#include "../FileSystem/FileSystem.h"
#include <cmath>
#include <fstream>
//==========================

namespace spartan
{
    void Animation::LoadFromFile(const std::string& file_path)
    {
        std::ifstream infile(file_path, std::ios::binary);
        if (!infile)
        {
            SP_LOG_ERROR("Failed to open animation file: %s", file_path.c_str());
            return;
        }

        SetResourceFilePath(file_path);

        uint32_t version = 0;
        infile.read(reinterpret_cast<char*>(&version), sizeof(uint32_t));
        if (version != 1)
        {
            SP_LOG_ERROR("Unsupported animation version: %u", version);
            return;
        }

        infile.read(reinterpret_cast<char*>(&m_duration_in_seconds), sizeof(float));
        infile.read(reinterpret_cast<char*>(&m_ticks_per_second), sizeof(double));

        uint32_t channel_count = 0;
        infile.read(reinterpret_cast<char*>(&channel_count), sizeof(uint32_t));

        m_channels.clear();
        m_channels.reserve(channel_count);

        for (uint32_t i = 0; i < channel_count; i++)
        {
            AnimationChannel channel;

            uint32_t name_len = 0;
            infile.read(reinterpret_cast<char*>(&name_len), sizeof(uint32_t));
            channel.bone_name.resize(name_len);
            infile.read(channel.bone_name.data(), name_len);

            auto read_keys_vec3 = [&](std::vector<AnimationKey<math::Vector3>>& keys) {
                uint32_t count = 0;
                infile.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
                keys.resize(count);
                for (uint32_t k = 0; k < count; k++)
                {
                    infile.read(reinterpret_cast<char*>(&keys[k].time), sizeof(float));
                    infile.read(reinterpret_cast<char*>(&keys[k].value.x), sizeof(float));
                    infile.read(reinterpret_cast<char*>(&keys[k].value.y), sizeof(float));
                    infile.read(reinterpret_cast<char*>(&keys[k].value.z), sizeof(float));
                }
            };

            auto read_keys_quat = [&](std::vector<AnimationKey<math::Quaternion>>& keys) {
                uint32_t count = 0;
                infile.read(reinterpret_cast<char*>(&count), sizeof(uint32_t));
                keys.resize(count);
                for (uint32_t k = 0; k < count; k++)
                {
                    infile.read(reinterpret_cast<char*>(&keys[k].time), sizeof(float));
                    infile.read(reinterpret_cast<char*>(&keys[k].value.x), sizeof(float));
                    infile.read(reinterpret_cast<char*>(&keys[k].value.y), sizeof(float));
                    infile.read(reinterpret_cast<char*>(&keys[k].value.z), sizeof(float));
                    infile.read(reinterpret_cast<char*>(&keys[k].value.w), sizeof(float));
                }
            };

            read_keys_vec3(channel.position_keys);
            read_keys_quat(channel.rotation_keys);
            read_keys_vec3(channel.scale_keys);

            m_channels.push_back(std::move(channel));
        }

        infile.close();
    }

    void Animation::SaveToFile(const std::string& file_path)
    {
        std::ofstream outfile(file_path, std::ios::binary);
        if (!outfile)
        {
            SP_LOG_ERROR("Failed to open file for writing: %s", file_path.c_str());
            return;
        }

        uint32_t version = 1;
        outfile.write(reinterpret_cast<const char*>(&version), sizeof(uint32_t));

        outfile.write(reinterpret_cast<const char*>(&m_duration_in_seconds), sizeof(float));
        outfile.write(reinterpret_cast<const char*>(&m_ticks_per_second), sizeof(double));

        uint32_t channel_count = static_cast<uint32_t>(m_channels.size());
        outfile.write(reinterpret_cast<const char*>(&channel_count), sizeof(uint32_t));

        for (const auto& channel : m_channels)
        {
            uint32_t name_len = static_cast<uint32_t>(channel.bone_name.size());
            outfile.write(reinterpret_cast<const char*>(&name_len), sizeof(uint32_t));
            outfile.write(channel.bone_name.data(), name_len);

            auto write_keys_vec3 = [&](const std::vector<AnimationKey<math::Vector3>>& keys) {
                uint32_t count = static_cast<uint32_t>(keys.size());
                outfile.write(reinterpret_cast<const char*>(&count), sizeof(uint32_t));
                for (const auto& key : keys)
                {
                    outfile.write(reinterpret_cast<const char*>(&key.time), sizeof(float));
                    outfile.write(reinterpret_cast<const char*>(&key.value.x), sizeof(float));
                    outfile.write(reinterpret_cast<const char*>(&key.value.y), sizeof(float));
                    outfile.write(reinterpret_cast<const char*>(&key.value.z), sizeof(float));
                }
            };

            auto write_keys_quat = [&](const std::vector<AnimationKey<math::Quaternion>>& keys) {
                uint32_t count = static_cast<uint32_t>(keys.size());
                outfile.write(reinterpret_cast<const char*>(&count), sizeof(uint32_t));
                for (const auto& key : keys)
                {
                    outfile.write(reinterpret_cast<const char*>(&key.time), sizeof(float));
                    outfile.write(reinterpret_cast<const char*>(&key.value.x), sizeof(float));
                    outfile.write(reinterpret_cast<const char*>(&key.value.y), sizeof(float));
                    outfile.write(reinterpret_cast<const char*>(&key.value.z), sizeof(float));
                    outfile.write(reinterpret_cast<const char*>(&key.value.w), sizeof(float));
                }
            };

            write_keys_vec3(channel.position_keys);
            write_keys_quat(channel.rotation_keys);
            write_keys_vec3(channel.scale_keys);
        }

        outfile.close();
    }

    void Animation::AddChannel(const std::string& bone_name,
                                const std::vector<AnimationKey<math::Vector3>>& position_keys,
                                const std::vector<AnimationKey<math::Quaternion>>& rotation_keys,
                                const std::vector<AnimationKey<math::Vector3>>& scale_keys)
    {
        AnimationChannel channel;
        channel.bone_name = bone_name;
        channel.position_keys = position_keys;
        channel.rotation_keys = rotation_keys;
        channel.scale_keys = scale_keys;
        m_channels.push_back(std::move(channel));
    }

    math::Vector3 Animation::LerpV3(const std::vector<AnimationKey<math::Vector3>>& keys, float t) const
    {
        if (keys.empty()) return math::Vector3::Zero;
        if (keys.size() == 1) return keys[0].value;

        // Find surrounding keyframes
        size_t index = 0;
        for (; index < keys.size() - 1; index++)
        {
            if (t < keys[index + 1].time)
                break;
        }

        const AnimationKey<math::Vector3>& key0 = keys[index];
        const size_t next_index = (index + 1 < keys.size()) ? index + 1 : index;
        const AnimationKey<math::Vector3>& key1 = keys[next_index];

        // Normalize t between key0 and key1
        float range = key1.time - key0.time;
        if (range < 0.0001f) return key0.value;
        float factor = (t - key0.time) / range;

        // Linear interpolation
        return math::Vector3::Lerp(key0.value, key1.value, factor);
    }

    math::Quaternion Animation::SlerpQ(const std::vector<AnimationKey<math::Quaternion>>& keys, float t) const
    {
        if (keys.empty()) return math::Quaternion::Identity;
        if (keys.size() == 1) return keys[0].value;

        // Find surrounding keyframes
        size_t index = 0;
        for (; index < keys.size() - 1; index++)
        {
            if (t < keys[index + 1].time)
                break;
        }

        const AnimationKey<math::Quaternion>& key0 = keys[index];
        const size_t next_index = (index + 1 < keys.size()) ? index + 1 : index;
        const AnimationKey<math::Quaternion>& key1 = keys[next_index];

        // Normalize t between key0 and key1
        float range = key1.time - key0.time;
        if (range < 0.0001f) return key0.value;
        float factor = (t - key0.time) / range;

        // SLERP interpolation
        // If the angle is too small, use Lerp to avoid division by zero
        const float dot = math::Quaternion::Dot(key0.value, key1.value);
        const float theta = std::acos(std::clamp(dot, -1.0f, 1.0f));

        if (theta < 0.001f)
        {
            return math::Quaternion::Lerp(key0.value, key1.value, factor).Normalized();
        }

        const float sin_theta = std::sin(theta);
        const float factor0 = std::sin((1.0f - factor) * theta) / sin_theta;
        const float factor1 = std::sin(factor * theta) / sin_theta;

        // Use the shortest path
        if (dot < 0.0f)
        {
            return (key0.value * factor0 - key1.value * factor1).Normalized();
        }

        return (key0.value * factor0 + key1.value * factor1).Normalized();
    }

    math::Matrix Animation::SampleBone(const std::string& bone_name, float time_ticks) const
    {
        // Find channel for this bone
        for (const auto& channel : m_channels)
        {
            if (channel.bone_name == bone_name)
            {
                // Sample position, rotation, scale at time_ticks
                math::Vector3 position = LerpV3(channel.position_keys, time_ticks);
                math::Quaternion rotation = SlerpQ(channel.rotation_keys, time_ticks);
                math::Vector3 scale = LerpV3(channel.scale_keys, time_ticks);

                // Build transformation matrix
                return math::Matrix::CreateRotation(rotation) * math::Matrix::CreateScale(scale) * math::Matrix::CreateTranslation(position);
            }
        }

        // Bone not found in animation channels
        return math::Matrix::Identity;
    }
}
