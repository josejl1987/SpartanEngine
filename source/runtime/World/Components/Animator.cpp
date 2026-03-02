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

//= INCLUDES ==================
#include "pch.h"
#include "Animator.h"
#include "Renderable.h"
#include "../Entity.h"
#include "../../Resource/ResourceCache.h"
#include "../../Geometry/Mesh.h"
#include "../../Core/Timer.h"
SP_WARNINGS_OFF
#include "../IO/pugixml.hpp"
SP_WARNINGS_ON
//=============================

namespace spartan
{
    Animator::Animator(Entity* entity)
        : Component(entity)
    {
        SP_REGISTER_ATTRIBUTE_VALUE_VALUE(m_animation_path, std::string);
        SP_REGISTER_ATTRIBUTE_VALUE_VALUE(m_is_playing, bool);
        SP_REGISTER_ATTRIBUTE_VALUE_VALUE(m_loop, bool);
        SP_REGISTER_ATTRIBUTE_VALUE_VALUE(m_speed, float);
        SP_REGISTER_ATTRIBUTE_VALUE_VALUE(m_animation_time, float);
    }

    void Animator::Initialize()
    {
        // Load animation resource from path
        if (!m_animation_path.empty())
        {
            SetAnimationByPath(m_animation_path);
        }
    }

    void Animator::Tick()
    {
        if (!m_is_playing || !m_animation)
            return;

        // Advance animation time
        float delta_time = static_cast<float>(Timer::GetDeltaTimeSec());
        m_animation_time += delta_time * m_speed;

        // Check for loop
        if (m_animation_time >= m_animation->GetDurationInSeconds())
        {
            if (m_loop)
            {
                m_animation_time = fmodf(m_animation_time, m_animation->GetDurationInSeconds());
            }
            else
            {
                m_animation_time = m_animation->GetDurationInSeconds();
                m_is_playing = false;
            }
        }

        // Update bone matrices
        UpdateBoneMatrices();
    }

    void Animator::UpdateBoneMatrices()
    {
        if (!m_animation)
            return;

        // Find renderable component (on self or any descendant)
        Renderable* renderable = GetEntity()->GetComponent<Renderable>();
        Entity* renderable_entity = GetEntity();
        if (!renderable)
        {
            // Check descendants for renderable
            std::vector<Entity*> descendants;
            GetEntity()->GetDescendants(&descendants);
            for (Entity* descendant : descendants)
            {
                renderable = descendant->GetComponent<Renderable>();
                if (renderable)
                {
                    renderable_entity = descendant;
                    break;
                }
            }
        }
        if (!renderable)
            return;

        const Mesh* mesh = renderable->GetMesh();
        if (!mesh || !mesh->IsSkinned())
            return;

        const BoneData* bone_data = mesh->GetBoneData();
        if (!bone_data)
            return;

        const uint32_t bone_count = bone_data->bone_count;

        // Resize bone matrix arrays
        m_bone_matrices_previous = std::move(m_bone_matrices_current);
        m_bone_matrices_current.resize(bone_count);
        if (m_bone_matrices_previous.size() != bone_count)
        {
            m_bone_matrices_previous.resize(bone_count, math::Matrix::Identity);
        }

        // Convert animation time to ticks
        float time_ticks = static_cast<float>(m_animation_time * m_animation->GetTicksPerSecond());

        // Sample animation for each bone
        for (uint32_t i = 0; i < bone_count; i++)
        {
            const std::string& bone_name = bone_data->bone_names[i];
            math::Matrix local_pose = m_animation->SampleBone(bone_name, time_ticks);

            // Apply bone offset transform: final = offset * local_pose
            m_bone_matrices_current[i] = bone_data->bone_offsets[i] * local_pose;
        }
    }

    void Animator::Play()
    {
        m_is_playing = true;
    }

    void Animator::Pause()
    {
        m_is_playing = false;
    }

    void Animator::Stop(bool reset)
    {
        m_is_playing = false;
        if (reset)
        {
            m_animation_time = 0.0f;
            UpdateBoneMatrices();
        }
    }

    void Animator::SetAnimationByPath(const std::string& path)
    {
        m_animation_path = path;

        // Load animation resource
        m_animation = ResourceCache::GetByPath<Animation>(path);
        if (!m_animation)
        {
            SP_LOG_WARNING("Failed to load animation: %s", path.c_str());
            m_animation_path.clear(); // Clear path on failure to indicate no animation loaded
        }
        else
        {
            // Initialize bone matrices
            UpdateBoneMatrices();
        }
    }

    void Animator::Save(pugi::xml_node& node)
    {
        node.append_attribute("animation_path") = m_animation_path.c_str();
        node.append_attribute("is_playing") = m_is_playing;
        node.append_attribute("loop") = m_loop;
        node.append_attribute("speed") = m_speed;
        node.append_attribute("animation_time") = m_animation_time;
    }

    void Animator::Load(pugi::xml_node& node)
    {
        m_animation_path = node.attribute("animation_path").as_string("");
        m_is_playing = node.attribute("is_playing").as_bool(false);
        m_loop = node.attribute("loop").as_bool(true);
        m_speed = node.attribute("speed").as_float(1.0f);
        m_animation_time = node.attribute("animation_time").as_float(0.0f);

        // Load animation resource
        if (!m_animation_path.empty())
        {
            SetAnimationByPath(m_animation_path);
        }
    }

    void Animator::RegisterForScripting(sol::state_view state)
    {
        auto new_usertype = state.new_usertype<Animator>("Animator",
            sol::base_classes, sol::bases<Component>());

        // Properties
        new_usertype["is_playing"] = sol::property(&Animator::IsPlaying, [] (Animator* anim, bool value) {
            if (value) anim->Play(); else anim->Pause();
        });
        new_usertype["loop"] = sol::property(&Animator::IsLooping, &Animator::SetLooping);
        new_usertype["speed"] = sol::property(&Animator::GetSpeed, &Animator::SetSpeed);
        new_usertype["animation_time"] = sol::property([] (Animator* anim) { return anim->m_animation_time; });
        new_usertype["animation_path"] = sol::property(&Animator::GetAnimationPath, &Animator::SetAnimationByPath);

        // Methods
        new_usertype["Play"] = &Animator::Play;
        new_usertype["Pause"] = &Animator::Pause;
        new_usertype["Stop"] = &Animator::Stop;
        new_usertype["SetAnimation"] = &Animator::SetAnimationByPath;
        new_usertype["GetBoneMatrices"] = &Animator::GetBoneMatrices;
    }
}
