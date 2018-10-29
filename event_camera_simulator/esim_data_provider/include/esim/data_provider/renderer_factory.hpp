#pragma once

#include <gflags/gflags.h>
#include <esim/rendering/renderer_base.hpp>
#include <esim/rendering/simple_renderer_base.hpp>

namespace event_camera_simulator {

bool loadPreprocessedImage(const std::string& path_to_img, cv::Mat *img);
Renderer::Ptr loadRendererFromGflags();
SimpleRenderer::Ptr loadSimpleRendererFromGflags();

} // namespace event_camera_simulator
