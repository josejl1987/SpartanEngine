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
#include "Breadcrumbs.h"
#include "../RHI/RHI_Buffer.h"
//=============================

namespace spartan
{
    void Breadcrumbs::Initialize()
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        // cpu markers
        m_markers.resize(max_markers);
        m_frame_index   = 0;
        m_current_index = 0;
        m_current_depth = 0;

        // gpu breadcrumb buffer - host visible and host coherent so the cpu can read it after a crash
        m_gpu_buffer = new RHI_Buffer(
            RHI_Buffer_Type::Storage,
            sizeof(uint32_t),
            max_gpu_markers,
            nullptr,
            true,
            "breadcrumb_gpu"
        );

        m_gpu_marker_count = 0;
        m_gpu_marker_names.fill(nullptr);
        m_gpu_marker_begin_to_slot.fill(-1);

        m_initialized = true;
    }

    void Breadcrumbs::Shutdown()
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        m_markers.clear();

        if (m_gpu_buffer)
        {
            delete m_gpu_buffer;
            m_gpu_buffer = nullptr;
        }

        m_gpu_marker_count = 0;
        m_initialized      = false;
    }

    int32_t Breadcrumbs::GpuMarkerBegin(const char* name)
    {
        if (!m_initialized || !m_gpu_buffer || !name)
            return -1;

        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_gpu_marker_count >= max_gpu_markers)
            return -1;

        uint32_t slot            = m_gpu_marker_count++;
        m_gpu_marker_names[slot] = name;

        return static_cast<int32_t>(slot);
    }

    void Breadcrumbs::GpuMarkerEnd(int32_t marker_index)
    {
        if (!m_initialized || marker_index < 0 || marker_index >= static_cast<int32_t>(max_gpu_markers))
            return;

        // nothing to track on the cpu side for end - the gpu buffer write handles it
    }

    void Breadcrumbs::ResetGpuMarkers()
    {
        if (!m_gpu_buffer)
            return;

        m_gpu_marker_count = 0;
        m_gpu_marker_names.fill(nullptr);
        m_gpu_marker_begin_to_slot.fill(-1);

        // zero out the mapped buffer so all slots read as "not reached"
        void* mapped = m_gpu_buffer->GetMappedData();
        if (mapped)
        {
            memset(mapped, 0, max_gpu_markers * sizeof(uint32_t));
        }
    }

    void Breadcrumbs::WriteGpuReport(std::string& report)
    {
        if (!m_gpu_buffer)
            return;

        uint32_t* data = static_cast<uint32_t*>(m_gpu_buffer->GetMappedData());
        if (!data)
            return;

        report += "\n=================================================\n";
        report += "GPU-SIDE BREADCRUMBS (execution trace on gpu)\n";
        report += "=================================================\n\n";

        // find the last slot the gpu wrote to
        int32_t last_started_slot  = -1;
        int32_t last_completed_slot = -1;
        for (int32_t i = static_cast<int32_t>(m_gpu_marker_count) - 1; i >= 0; i--)
        {
            if (data[i] != 0 && last_started_slot < 0)
            {
                last_started_slot = i;
            }
            if (data[i] == gpu_marker_completed && last_completed_slot < 0)
            {
                last_completed_slot = i;
            }
            if (last_started_slot >= 0 && last_completed_slot >= 0)
                break;
        }

        if (last_started_slot < 0)
        {
            report += "no gpu markers were reached - the crash may have\n";
            report += "occurred before the first tracked operation.\n";
            return;
        }

        for (uint32_t i = 0; i < m_gpu_marker_count; i++)
        {
            const char* name = m_gpu_marker_names[i];
            uint32_t value   = data[i];

            if (!name)
                continue;

            report += "  marker " + std::to_string(i) + ": " + name;

            if (value == 0)
            {
                report += "  [not reached]\n";
            }
            else if (value == gpu_marker_completed)
            {
                report += "  [completed]\n";
            }
            else
            {
                // started but not completed - this is a candidate for the crash location
                bool is_last_started = (static_cast<int32_t>(i) == last_started_slot);
                if (is_last_started)
                {
                    report += "  [in progress - LIKELY CULPRIT]\n";
                }
                else
                {
                    report += "  [started]\n";
                }
            }
        }
    }

    void Breadcrumbs::WriteReport()
    {
        std::string report;
        report.reserve(8192);

        report += "=================================================\n";
        report += "GPU CRASH REPORT - Breadcrumbs\n";
        report += "=================================================\n\n";

        // cpu-side markers
        std::vector<const Marker*> incomplete_markers;
        for (const auto& marker : m_markers)
        {
            if (marker.state == MarkerState::Started)
            {
                incomplete_markers.push_back(&marker);
            }
        }

        report += "CPU-SIDE MARKERS:\n";
        report += "-------------------------------------------------\n\n";

        if (incomplete_markers.empty())
        {
            report += "no incomplete cpu markers found.\n";
        }
        else
        {
            report += "incomplete markers (started but never completed):\n\n";

            std::sort(incomplete_markers.begin(), incomplete_markers.end(),
                [](const Marker* a, const Marker* b)
                {
                    if (a->frame_index != b->frame_index)
                        return a->frame_index < b->frame_index;
                    return a->depth < b->depth;
                });

            for (const auto* marker : incomplete_markers)
            {
                auto now  = std::chrono::steady_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - marker->start_time);

                for (uint32_t d = 0; d < marker->depth; d++)
                    report += "  ";

                report += "-> frame " + std::to_string(marker->frame_index);
                report += " | " + std::string(marker->name.data());
                report += " | running for: " + std::to_string(diff.count()) + "ms\n";
            }
        }

        // gpu-side markers
        WriteGpuReport(report);

        report += "\n=================================================\n";
        report += "the gpu-side markers show the exact point where the\n";
        report += "gpu stopped executing. the last 'in progress' marker\n";
        report += "is the most likely cause of the crash.\n";
        report += "=================================================\n";

        std::ofstream file("gpu_crash.txt", std::ios::binary);
        if (file.good())
        {
            file.write(report.c_str(), report.size());
            file.close();
        }
    }
}
