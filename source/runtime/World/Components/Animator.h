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

//= INCLUDES ==================
#include "Component.h"
#include "../../Rendering/Animation.h"
//=============================

namespace spartan
{
    class Animator : public Component
    {
    public:
        Animator(Entity* entity);
        ~Animator() = default;

        // Component
        void Initialize() override;
        void Tick() override;
        void Save(pugi::xml_node& node) override;
        void Load(pugi::xml_node& node) override;

        // Animation playback control
        void Play();
        void Pause();
        void Stop(bool reset = true);

        // Animation resource
        void SetAnimationByPath(const std::string& path);
        const std::string& GetAnimationPath() const { return m_animation_path; }

        // Playback properties
        void SetLooping(bool loop) { m_loop = loop; }
        bool IsLooping() const { return m_loop; }
        void SetSpeed(float speed) { m_speed = speed; }
        float GetSpeed() const { return m_speed; }
        bool IsPlaying() const { return m_is_playing; }
        float GetAnimationTime() const { return m_animation_time; }

        // Bone matrices for GPU skinning
        const std::vector<math::Matrix>& GetBoneMatrices() const { return m_bone_matrices_current; }
        const std::vector<math::Matrix>& GetBoneMatricesPrev() const { return m_bone_matrices_previous; }

        // Scripting integration
        static void RegisterForScripting(sol::state_view state);

    private:
        void UpdateBoneMatrices();

        // Animation resource
        std::string m_animation_path;
        std::shared_ptr<Animation> m_animation;

        // Playback state
        bool m_is_playing = false;
        bool m_loop = true;
        float m_speed = 1.0f;
        float m_animation_time = 0.0f;

        // Bone matrices (current and previous frame for motion vectors)
        std::vector<math::Matrix> m_bone_matrices_current;
        std::vector<math::Matrix> m_bone_matrices_previous;
    };
}
