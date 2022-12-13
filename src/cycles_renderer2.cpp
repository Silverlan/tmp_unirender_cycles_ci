
/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#include "util_raytracing/scene.hpp"
#include "util_raytracing/mesh.hpp"
#include "util_raytracing/object.hpp"
#include "util_raytracing/scene.hpp"
#include "util_raytracing/light.hpp"
#include "util_raytracing/camera.hpp"
#include "unirender/cycles/baking.hpp"
#include "unirender/cycles/renderer.hpp"
#include "unirender/cycles/display_driver.hpp"
#include "util_raytracing/shader_nodes.hpp"
#include "unirender/cycles/ccl_shader.hpp"
#include "util_raytracing/model_cache.hpp"
#include "util_raytracing/color_management.hpp"
#include "util_raytracing/denoise.hpp"
#include <sharedutils/util_hair.hpp>
#include <sharedutils/util_baking.hpp>
#include <fsys/ifile.hpp>
#include <util_ocio.hpp>
#include <scene/light.h>
#include <scene/camera.h>
#include <mathutil/umath_lighting.hpp>
#include <mathutil/units.h>
#include <session/buffers.h>
#include <scene/scene.h>
#include <session/session.h>
#include <scene/shader.h>
#include <scene/mesh.h>
#include <scene/shader_graph.h>
#include <scene/shader_nodes.h>
#include <scene/object.h>
#include <scene/background.h>
#include <scene/integrator.h>
#include <scene/svm.h>
#include <scene/bake.h>
#include <scene/particles.h>
#include <scene/hair.h>
#include <util/path.h>
// #define ENABLE_CYCLES_LOGGING
#ifdef ENABLE_CYCLES_LOGGING
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#endif
#include <util_image_buffer.hpp>

#ifdef _WIN32
#include <Shlobj.h>
#endif

static void validate_session(ccl::Scene &scene)
{
	for(auto *shader : scene.shaders)
	{
		if(shader->graph == nullptr)
			throw std::logic_error{"Found shader with invalid graph!"};
	}
}

// When baking, some pixels may end up with NaN values due to unknown issues
// with Cycles internally. We'll fix those pixels here by overwriting them with the average
// value of the surrounding pixels.
// This only happens in the direct lighting map.
static void fix_nan_pixels(uimg::ImageBuffer &imgBuf)
{
	auto w = imgBuf.GetWidth();
	auto h = imgBuf.GetHeight();
	for(auto x=decltype(w){0u};x<w;++x)
	{
		for(auto y=decltype(h){0u};y<h;++y)
		{
			auto view = imgBuf.GetPixelView(x,y);
			Vector3 color {
				view.GetFloatValue(uimg::Channel::R),
				view.GetFloatValue(uimg::Channel::G),
				view.GetFloatValue(uimg::Channel::B)
			};
			if(std::isnan(color.x) || std::isnan(color.y) || std::isnan(color.z))
			{
				std::vector<Vector4> neighborValues;
				auto addNeighborValue = [&neighborValues,&imgBuf,w,h](uint32_t x,uint32_t y) {
					if(x >= w || y >= h)
						return;
					auto view = imgBuf.GetPixelView(x,y);
					Vector4 color {
						view.GetFloatValue(uimg::Channel::R),
						view.GetFloatValue(uimg::Channel::G),
						view.GetFloatValue(uimg::Channel::B),
						view.GetFloatValue(uimg::Channel::A)
					};
					if(
						std::isnan(color[0]) || std::isnan(color[1]) || std::isnan(color[2]) || std::isnan(color[3]) ||
						color[3] == 0.f
					)
						return;
					neighborValues.push_back(color);
				};

				addNeighborValue(x -1,y);
				addNeighborValue(x -1,y -1);
				addNeighborValue(x -1,y +1);

				addNeighborValue(x +1,y);
				addNeighborValue(x +1,y -1);
				addNeighborValue(x +1,y +1);

				addNeighborValue(x,y -1);
				addNeighborValue(x -1,y -1);
				addNeighborValue(x +1,y -1);

				addNeighborValue(x,y +1);
				addNeighborValue(x -1,y +1);
				addNeighborValue(x +1,y +1);

				Vector4 avg{};
				for(auto &p : neighborValues)
					avg += p;
				if(!neighborValues.empty())
					avg = avg /static_cast<float>(neighborValues.size());

				view.SetValue(uimg::Channel::R,avg.x);
				view.SetValue(uimg::Channel::G,avg.y);
				view.SetValue(uimg::Channel::B,avg.z);
			}
		}
	}
}

#if 0
static void debug_dump_images(std::unordered_map<std::string,std::array<std::shared_ptr<uimg::ImageBuffer>,umath::to_integral(unirender::Renderer::StereoEye::Count)>> &resultImageBuffers)
{
	for(auto &pair : resultImageBuffers)
	{
		if(pair.second.empty())
			continue;
		auto &imgBuf = pair.second.front();
		if(!imgBuf)
			continue;
		filemanager::create_path("temp/cycles/");
		auto f = filemanager::open_file(
			"temp/cycles/debug_output_" +pair.first +".hdr",filemanager::FileMode::Write | filemanager::FileMode::Binary
		);
		if(f)
		{
			fsys::File fp {f};
			uimg::save_image(fp,*imgBuf,uimg::ImageFormat::HDR,1.f);
		}
	}
}
#endif

util::EventReply unirender::cycles::Renderer::HandleRenderStage(RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye eyeStage,unirender::Renderer::RenderStageResult *optResult)
{
	auto handled = unirender::Renderer::HandleRenderStage(worker,stage,eyeStage,optResult);
	if(handled == util::EventReply::Handled)
		return handled;
	switch(stage)
	{
	case ImageRenderStage::InitializeScene:
	{
		worker.AddThread([this,&worker]() {
			PrepareCyclesSceneForRendering();
			std::string err;
			if(!Initialize(*m_scene,err))
			{
				worker.Cancel("Failed to initialize scene: " +err);
				return RenderStageResult::Complete;
			}

			//if(Scene::IsRenderSceneMode(m_renderMode))
			{
				auto &cam = m_scene->GetCamera();
				auto stereoscopic = cam.IsStereoscopic();
				if(stereoscopic)
					m_cclScene->camera->set_stereo_eye(ccl::Camera::StereoEye::STEREO_LEFT);
				ImageRenderStage initialRenderStage;
				switch(m_renderMode)
				{
				case Scene::RenderMode::SceneAlbedo:
					initialRenderStage = ImageRenderStage::SceneAlbedo;
					break;
				case Scene::RenderMode::SceneNormals:
					initialRenderStage = ImageRenderStage::SceneNormals;
					break;
				case Scene::RenderMode::SceneDepth:
					initialRenderStage = ImageRenderStage::SceneDepth;
					break;
				default:
					initialRenderStage = ImageRenderStage::Lighting;
					break;
				//default:
				//	throw std::invalid_argument{"Invalid render mode " +std::to_string(umath::to_integral(m_renderMode))};
				}
				StartNextRenderStage(worker,initialRenderStage,stereoscopic ? StereoEye::Left : StereoEye::None);
				worker.Start();
			}
			/*else
			{
				StartNextRenderStage(worker,ImageRenderStage::Bake,StereoEye::None);
				worker.Start();
			}*/
			return RenderStageResult::Continue;
		});
		break;
	}
	case ImageRenderStage::Bake:
	{
		// Unused
		// StartTextureBaking(worker);
		break;
	}
	case ImageRenderStage::Lighting:
	case ImageRenderStage::SceneAlbedo:
	case ImageRenderStage::SceneNormals:
	case ImageRenderStage::SceneDepth:
	{
		if(umath::is_flag_set(m_stateFlags,StateFlags::ProgressiveRefine))
			m_progressiveRunning = true;
		worker.AddThread([this,&worker,stage,eyeStage]() {
			validate_session(*m_cclScene);

			InitStereoEye(eyeStage);
			if(umath::is_flag_set(m_stateFlags,StateFlags::SessionWasStarted))
			{
				//options.session->scene->camera->need_flags_update = true;
				//options.session->scene->camera->need_device_update = true;
				//options.session->reset(options.session_params, session_buffer_params());
				m_cclSession->reset(m_sessionParams,m_bufferParams);
				m_cclSession->start();
			}
			else
			{
				umath::set_flag(m_stateFlags,StateFlags::SessionWasStarted);
				m_cclSession->start();
			}

			auto progressMultiplier = umath::is_flag_set(m_stateFlags,StateFlags::NativeDenoising) ? 0.95f : 1.f;
			WaitForRenderStage(worker,0.f,progressMultiplier,[this,&worker,stage,eyeStage]() mutable -> RenderStageResult {
				if(umath::is_flag_set(m_stateFlags,StateFlags::ProgressiveRefine) == false)
					m_cclSession->wait();
				else if(m_progressiveRunning)
				{
					std::unique_lock<std::mutex> lock {m_progressiveMutex};
					m_progressiveCondition.wait(lock);
				}

				auto &resultImgBuf = GetResultImageBuffer(OUTPUT_COLOR,eyeStage);
				if(m_bakeData)
				{
					auto renderMode = m_scene->GetRenderMode();
					switch(renderMode)
					{
					case Scene::RenderMode::BakeAmbientOcclusion:
						resultImgBuf = GetOutputDriver()->GetImageBuffer(OUTPUT_AO);
						break;
					case Scene::RenderMode::BakeDiffuseLighting:
						GetResultImageBuffer(OUTPUT_DIFFUSE) = GetOutputDriver()->GetImageBuffer(OUTPUT_DIFFUSE);
						break;
					case Scene::RenderMode::BakeDiffuseLightingSeparate:
						GetResultImageBuffer(OUTPUT_DIFFUSE_DIRECT) = GetOutputDriver()->GetImageBuffer(OUTPUT_DIFFUSE_DIRECT);
						GetResultImageBuffer(OUTPUT_DIFFUSE_INDIRECT) = GetOutputDriver()->GetImageBuffer(OUTPUT_DIFFUSE_INDIRECT);
						break;
					default:
						resultImgBuf = GetOutputDriver()->GetImageBuffer(OUTPUT_COLOR);
						break;
					}

					if(Scene::IsLightmapRenderMode(renderMode))
					{
						worker.SetResultMessage("Fixing outliers...");
						auto &imgBuf = GetResultImageBuffer(OUTPUT_DIFFUSE_DIRECT);
						if(imgBuf)
							fix_nan_pixels(*imgBuf);

						worker.SetResultMessage("Baking margin...");
						for(auto &pair : m_resultImageBuffers)
						{
							for(auto &imgBuf : pair.second)
							{
								if(!imgBuf)
									continue;
								imgBuf->Convert(uimg::Format::RGBA_FLOAT);

								// Apply margin
								auto numPixels = imgBuf->GetPixelCount();
								std::vector<uint8_t> mask_buffer {};
								mask_buffer.resize(numPixels);
								constexpr auto margin = 16u;
								util::baking::fill_bake_mask(m_bakePixels, numPixels, reinterpret_cast<char*>(mask_buffer.data()));
								uimg::bake_margin(*imgBuf, mask_buffer, margin);
							}
						}
					}
				}
				else
				{
					switch(stage)
					{
					case ImageRenderStage::SceneAlbedo:
						resultImgBuf = GetOutputDriver()->GetImageBuffer(OUTPUT_ALBEDO);
						break;
					case ImageRenderStage::SceneNormals:
						resultImgBuf = GetOutputDriver()->GetImageBuffer(OUTPUT_NORMAL);
						break;
					case ImageRenderStage::SceneDepth:
						resultImgBuf = GetOutputDriver()->GetImageBuffer(OUTPUT_DEPTH);
						break;
					default:
						resultImgBuf = GetOutputDriver()->GetImageBuffer(OUTPUT_COLOR);
						break;
					}
				}
				
				for(auto &pair : m_resultImageBuffers)
				{
					for(auto &imgBuf : pair.second)
					{
						if(!imgBuf)
							continue;
						imgBuf->ToHDR();
					}
				}

				// ApplyPostProcessing(*resultImageBuffer,m_renderMode);

				auto tmpEyeStage = eyeStage;
				if(UpdateStereoEye(worker,stage,tmpEyeStage))
				{
					GetOutputDriver()->Reset();
					worker.Start(); // Lighting stage for the left eye is triggered by the user, but we have to start it manually for the right eye
					return RenderStageResult::Continue;
				}

				if(stage != ImageRenderStage::Lighting || !umath::is_flag_set(m_stateFlags,StateFlags::NativeDenoising))
					return StartNextRenderStage(worker,ImageRenderStage::FinalizeImage,eyeStage);

				auto &albedoImageBuffer = GetResultImageBuffer(OUTPUT_ALBEDO,eyeStage);
				auto &normalImageBuffer = GetResultImageBuffer(OUTPUT_NORMAL,eyeStage);
				albedoImageBuffer = GetOutputDriver()->GetImageBuffer(OUTPUT_ALBEDO);
				normalImageBuffer = GetOutputDriver()->GetImageBuffer(OUTPUT_NORMAL);
				assert(albedoImageBuffer != nullptr);
				assert(normalImageBuffer != nullptr);
				albedoImageBuffer->ToHDR();
				normalImageBuffer->ToHDR();

				std::string debugPass;
				GetApiData().GetFromPath("debug/returnPassAsResult")(debugPass);
				if(!debugPass.empty())
				{
					auto *imgBuf = FindResultImageBuffer(debugPass,eyeStage);
					if(imgBuf)
						GetResultImageBuffer(OUTPUT_COLOR,eyeStage) = imgBuf->Copy(uimg::Format::RGBA_FLOAT);
					return StartNextRenderStage(worker,ImageRenderStage::FinalizeImage,eyeStage);
				}

				return StartNextRenderStage(worker,ImageRenderStage::Denoise,eyeStage);
			});
		});
		break;
	}
	case ImageRenderStage::Albedo:
	{
		ReloadProgressiveRender();
		// Render albedo colors (required for denoising)
		m_renderMode = Scene::RenderMode::SceneAlbedo;
		if(eyeStage == StereoEye::Left || eyeStage == StereoEye::None)
			InitializeAlbedoPass(true);
		worker.AddThread([this,&worker,eyeStage,stage]() {
			validate_session(*m_cclScene);
			m_cclSession->start();
			WaitForRenderStage(worker,0.95f,0.025f,[this,&worker,eyeStage,stage]() mutable -> RenderStageResult {
				m_cclSession->wait();
				auto &albedoImageBuffer = GetResultImageBuffer(OUTPUT_ALBEDO,eyeStage);
				albedoImageBuffer = GetOutputDriver()->GetImageBuffer(OUTPUT_ALBEDO);
				// ApplyPostProcessing(*albedoImageBuffer,m_renderMode);

				auto tmpEyeStage = eyeStage;
				if(UpdateStereoEye(worker,stage,tmpEyeStage))
					return RenderStageResult::Continue;

				return StartNextRenderStage(worker,ImageRenderStage::Normal,eyeStage);
			});
		});
		worker.Start();
		break;
	}
	case ImageRenderStage::Normal:
	{
		ReloadProgressiveRender();
		// Render normals (required for denoising)
		m_renderMode = Scene::RenderMode::SceneNormals;
		if(eyeStage == StereoEye::Left || eyeStage == StereoEye::None)
			InitializeNormalPass(true);
		worker.AddThread([this,&worker,eyeStage,stage]() {
			validate_session(*m_cclScene);
			m_cclSession->start();
			WaitForRenderStage(worker,0.975f,0.025f,[this,&worker,eyeStage,stage]() mutable -> RenderStageResult {
				m_cclSession->wait();
				auto &normalImageBuffer = GetResultImageBuffer(OUTPUT_NORMAL,eyeStage);
				normalImageBuffer = GetOutputDriver()->GetImageBuffer(OUTPUT_NORMAL);
				// ApplyPostProcessing(*normalImageBuffer,m_renderMode);

				auto tmpEyeStage = eyeStage;
				if(UpdateStereoEye(worker,stage,tmpEyeStage))
					return RenderStageResult::Continue;

				return StartNextRenderStage(worker,ImageRenderStage::Denoise,eyeStage);
			});
		});
		worker.Start();
		break;
	}
	}
	if(optResult)
		*optResult = unirender::Renderer::RenderStageResult::Continue;
	return util::EventReply::Handled;
}

void unirender::cycles::Renderer::WaitForRenderStage(RenderWorker &worker,float baseProgress,float progressMultiplier,const std::function<RenderStageResult()> &fOnComplete)
{
	for(;;)
	{
		worker.UpdateProgress(baseProgress +umath::min(m_cclSession->progress.get_progress(),1.0) *progressMultiplier);

		if(worker.IsCancelled())
			SetCancelled("Cancelled by application.");

		if(m_restartState != 0)
		{
			while(m_restartState != 2);
			m_restartState = 0;
			continue;
		}

		if(umath::is_flag_set(m_stateFlags,StateFlags::ReloadSessionScheduled) && (!m_displayDriver || m_displayDriver->WasTileWritten()))
		{
			umath::set_flag(m_stateFlags,StateFlags::ReloadSessionScheduled,false);
			m_cclScene->camera->need_flags_update = true;
			m_cclScene->camera->need_device_update = true;
			m_cclSession->reset(m_sessionParams,m_bufferParams);
			if(m_displayDriver)
				m_displayDriver->ResetTileWrittenFlag();
		}

		if(umath::is_flag_set(m_stateFlags,StateFlags::ProgressiveRefine) && m_progressiveRunning == false)
			break;
		if(m_cclSession->progress.get_cancel())
		{
			if(m_restartState != 0)
				continue;
			std::cerr<<"WARNING: Cycles rendering has been cancelled: "<<m_cclSession->progress.get_cancel_message()<<std::endl;
			worker.Cancel(m_cclSession->progress.get_cancel_message(),1 /* error code */);
			break;
		}
		if(m_cclSession->progress.get_error())
		{
			std::string status;
			std::string subStatus;
			m_cclSession->progress.get_status(status,subStatus);
			std::cerr<<"WARNING: Cycles rendering has failed at status '"<<status<<"' ("<<subStatus<<") with error: "<<m_cclSession->progress.get_error_message()<<std::endl;
			worker.SetStatus(util::JobStatus::Failed,m_cclSession->progress.get_error_message());
			break;
		}
		std::string status,subStatus;
		m_cclSession->progress.get_status(status,subStatus);
		if(umath::min(m_cclSession->progress.get_progress(),1.0) == 1.0 || status == "Finished")
			break;
		std::this_thread::sleep_for(std::chrono::milliseconds{100});
	}
	if(worker.GetStatus() == util::JobStatus::Pending && fOnComplete != nullptr && fOnComplete() == RenderStageResult::Continue)
		return;
	if(worker.GetStatus() == util::JobStatus::Pending)
		worker.SetStatus(util::JobStatus::Successful);
}

void unirender::cycles::Renderer::InitStereoEye(StereoEye eyeStage)
{
	if(eyeStage == StereoEye::Left)
	{
		// Switch to right eye
		m_cclScene->camera->set_stereo_eye(ccl::Camera::StereoEye::STEREO_RIGHT);
		m_cclScene->camera->need_flags_update = true;
		m_cclScene->camera->need_device_update = true;
	}
	else if(eyeStage == StereoEye::Right)
	{
		// Switch back to left eye and continue with next stage
		m_cclScene->camera->set_stereo_eye(ccl::Camera::StereoEye::STEREO_LEFT);
		m_cclScene->camera->need_flags_update = true;
		m_cclScene->camera->need_device_update = true;
	}
}

bool unirender::cycles::Renderer::UpdateStereoEye(unirender::RenderWorker &worker,ImageRenderStage stage,StereoEye &eyeStage)
{
	if(eyeStage == StereoEye::Left)
	{
		// Switch to right eye
		ReloadProgressiveRender(false);
		StartNextRenderStage(worker,stage,StereoEye::Right);
		return true;
	}
	else if(eyeStage == StereoEye::Right)
	{
		// Switch back to left eye and continue with next stage
		eyeStage = StereoEye::Left;
	}
	return false;
}

int unirender::cycles::Renderer::GetTileSize() const {return m_cclSession->params.tile_size;}

void unirender::cycles::Renderer::ReloadProgressiveRender(bool clearExposure,bool waitForPreviousCompletion)
{
	auto &createInfo = m_scene->GetCreateInfo();
	if(createInfo.progressive == false)
		return;
	m_tileManager.Reload(waitForPreviousCompletion);
	if(clearExposure)
		m_tileManager.SetExposure(1.f);
	umath::set_flag(m_stateFlags,StateFlags::ProgressiveRefine,false);
	m_cclSession->progress.reset();
}

void unirender::cycles::Renderer::PrepareCyclesSceneForRendering()
{
	unirender::Renderer::PrepareCyclesSceneForRendering();
}

void unirender::cycles::Renderer::SetCancelled(const std::string &msg)
{
	std::scoped_lock lock {m_cancelMutex};
	if(m_cancelled || m_cclSession == nullptr)
		return;
	m_cancelled = true;
	m_cclSession->progress.set_cancel(msg);
	m_cclSession->cancel(true);
	m_tileManager.Cancel();
}

void unirender::cycles::Renderer::CloseCyclesScene()
{
	m_renderData = {};
	m_tileManager.StopAndWait();
	m_bakeData = nullptr;

	if(m_cclSession == nullptr)
		return;
	m_cclSession = nullptr;
}

bool unirender::cycles::Renderer::InitializeBakingData()
{
	auto *bakeName = m_scene->GetBakeTargetName();
	if(!bakeName)
		return false;
	auto resolution = m_scene->GetResolution();
	auto imgWidth = resolution.x;
	auto imgHeight = resolution.y;
	auto numPixels = imgWidth *imgHeight;
	auto *o = FindObject(*bakeName);
	auto *aoTarget = o ? FindCclObject(*o) : nullptr;
	if(aoTarget == nullptr)
		return false;
	m_bakeData = std::make_unique<util::baking::BakeDataView>();
	m_bakeData->bakeImageWidth = imgWidth;
	m_bakeData->bakeImageHeight = imgHeight;
	auto &pixelArray = m_bakePixels;
	pixelArray.resize(numPixels);
	m_bakeData->bakePixels = m_bakePixels.data();
	auto bakeLightmaps = Scene::IsLightmapRenderMode(m_renderMode);

	baking::prepare_bake_data(*this,*o,pixelArray.data(),numPixels,imgWidth,imgHeight,bakeLightmaps);

	auto objectId = aoTarget ? FindCCLObjectId(*aoTarget) : std::optional<uint32_t>{};
	if(!objectId.has_value())
		return false;
	m_bakeData->objectId = *objectId;
	return true;
}
void unirender::cycles::Renderer::StartTextureBaking(RenderWorker &worker)
{
#if 0
	// Baking cannot be done with cycles directly, we will have to
	// do some additional steps first.
	worker.AddThread([this,&worker]() {
		// InitializeAlbedoPass(true);
		//auto bufferParams = m_cclSession->buffers->params;
		auto resolution = m_scene->GetResolution();
		auto imgWidth = resolution.x;
		auto imgHeight = resolution.y;
		//m_cclScene->bake_manager->set_baking(true);
		//m_cclSession->load_kernels();

		static auto albedoOnly = false;
		switch(m_renderMode)
		{
		case Scene::RenderMode::BakeAmbientOcclusion:
		case Scene::RenderMode::BakeDiffuseLighting:
			/*if(albedoOnly)
				ccl::Pass::add(ccl::PASS_DIFFUSE_COLOR,m_cclScene->film->passes);
			else
				ccl::Pass::add(ccl::PASS_LIGHT,m_cclScene->film->passes);*/
			break;
		case Scene::RenderMode::BakeNormals:
			break;
		}
		
		auto bufferParams = GetBufferParameters();
		if(albedoOnly)
		{
			ccl::vector<ccl::Pass> passes;
			auto displayPass = ccl::PassType::PASS_DIFFUSE_COLOR;
			ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined");
			displayPass = ccl::PassType::PASS_COMBINED;
			//bufferParams.passes = passes;
			auto *scene = m_cclScene;
			auto &film = *m_cclScene->film;
			film.tag_passes_update(scene, passes);
			film.display_pass = displayPass;
			//film.tag_update(&scene);
			//scene.integrator->tag_update(&scene);
		}
		else
		{
			ccl::vector<ccl::Pass> passes;
			if(m_renderMode == Scene::RenderMode::BakeDiffuseLighting)
			{
				//ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined");
				//ccl::Pass::add(ccl::PassType::PASS_LIGHT,passes,"light");
				ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined");
				ccl::Pass::add(ccl::PassType::PASS_LIGHT,passes,"light");
				//ccl::Pass::add(ccl::PassType::PASS_DIFFUSE_COLOR,passes,"diffuse_color");
				//ccl::Pass::add(ccl::PassType::PASS_DIFFUSE_DIRECT,passes,"diffuse_direct");
				//ccl::Pass::add(ccl::PassType::PASS_DIFFUSE_INDIRECT,passes,"diffuse_indirect");
				m_cclScene->film->display_pass = ccl::PassType::PASS_COMBINED;
			}
			else
			{
				auto &scene = m_cclScene;
				auto &film = *m_cclScene->film;
				ccl::Pass::add(ccl::PassType::PASS_AO,passes,"ao");
				ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
				film.display_pass = ccl::PassType::PASS_AO;
			}
			auto &scene = m_cclScene;
			auto &film = *m_cclScene->film;
			film.passes = passes;
			bufferParams.passes = passes;
			film.tag_passes_update(scene, bufferParams.passes);
			// film.passes
			/*ccl::vector<ccl::Pass> passes;
			ccl::Pass::add(ccl::PassType::PASS_DIFFUSE_DIRECT,passes,"diffuse_direct");
			ccl::Pass::add(ccl::PassType::PASS_DIFFUSE_INDIRECT,passes,"diffuse_indirect");
			ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
			auto displayPass = ccl::PassType::PASS_COMBINED;
			
			bufferParams.passes = passes;
			auto &scene = m_cclScene;
			auto &film = *m_cclScene.film;
			film.tag_passes_update(&scene, passes);
			film.display_pass = displayPass;*/
		}

		// Multiple importance sampling doesn't work properly with lightmap baking
		// and causes hard light edges
		for(auto *l : m_cclScene->lights)
		{
			l->set_use_mis(false);
			l->tag_update(m_cclScene);
		}

		m_cclScene->film->tag_update(m_cclScene);
		m_cclScene->integrator->tag_update(m_cclScene);
		
		// TODO: Shader limits are arbitrarily chosen, check how Blender does it?
		m_cclScene->bake_manager->set_shader_limit(256,256);
		m_cclSession->tile_manager.set_samples(m_cclSession->params.samples);
		
		ccl::ShaderEvalType shaderType;
		int bake_pass_filter;
		switch(m_renderMode)
		{
		case Scene::RenderMode::BakeAmbientOcclusion:
			shaderType = ccl::ShaderEvalType::SHADER_EVAL_AO;
			bake_pass_filter = ccl::BAKE_FILTER_AO;
			break;
		case Scene::RenderMode::BakeDiffuseLighting:
			/*if(albedoOnly)
			{
				shaderType = ccl::ShaderEvalType::SHADER_EVAL_DIFFUSE;
				bake_pass_filter = ccl::BAKE_FILTER_DIFFUSE | ccl::BAKE_FILTER_COLOR;
			}
			else
			{
				shaderType = ccl::ShaderEvalType::SHADER_EVAL_DIFFUSE;//ccl::ShaderEvalType::SHADER_EVAL_DIFFUSE;
				bake_pass_filter = 510 &~ccl::BAKE_FILTER_COLOR &~ccl::BAKE_FILTER_GLOSSY;//ccl::BakePassFilterCombos::BAKE_FILTER_COMBINED;//ccl::BAKE_FILTER_DIFFUSE | ccl::BAKE_FILTER_INDIRECT | ccl::BAKE_FILTER_DIRECT;
			}*/
			//shaderType = ccl::ShaderEvalType::SHADER_EVAL_DIFFUSE;
			shaderType = ccl::ShaderEvalType::SHADER_EVAL_COMBINED;//ccl::ShaderEvalType::SHADER_EVAL_DIFFUSE;
			bake_pass_filter = 255;//255 &~ccl::BAKE_FILTER_COLOR;//ccl::BAKE_FILTER_DIFFUSE;
			break;
		case Scene::RenderMode::BakeNormals:
			shaderType = ccl::ShaderEvalType::SHADER_EVAL_NORMAL;
			bake_pass_filter = 0;
			break;
		}
		if(m_renderMode != Scene::RenderMode::BakeDiffuseLighting)
			bake_pass_filter = ccl::BakeManager::shader_type_to_pass_filter(shaderType,bake_pass_filter);
		if(worker.IsCancelled())
			return;

		auto numPixels = imgWidth *imgHeight;
		auto *o = FindObject("bake_target");
		auto *aoTarget = o ? FindCclObject(*o) : nullptr;
		if(aoTarget == nullptr)
		{
			worker.SetStatus(util::JobStatus::Failed,"Invalid bake target!");
			return;
		}
		std::vector<unirender::baking::BakePixel> pixelArray;
		pixelArray.resize(numPixels);
		auto bakeLightmaps = (m_renderMode == Scene::RenderMode::BakeDiffuseLighting);

		unirender::baking::prepare_bake_data(*this,*o,pixelArray.data(),numPixels,imgWidth,imgHeight,bakeLightmaps);
		
		if(worker.IsCancelled())
			return;

		auto objectId = aoTarget ? FindCCLObjectId(*aoTarget) : std::optional<uint32_t>{};

		//FindObject
		assert(objectId.has_value());
		ccl::BakeData *bake_data = NULL;
		uint32_t triOffset = 0u;
		// Note: This has been commented because it can cause crashes in some cases. To fix the underlying issue, the mesh for
		// which ao should be baked is now moved to the front so it always has a triangle offset of 0 (See 'SetAOBakeTarget'.).
		/// It would be expected that the triangle offset is relative to the object, but that's not actually the case.
		/// Instead, it seems to be a global triangle offset, so we have to count the number of triangles for all objects
		/// before this one.
		///for(auto i=decltype(objectId){0u};i<objectId;++i)
		///	triOffset += m_objects.at(i)->GetMesh().GetTriangleCount();
		bake_data = m_cclScene->bake_manager->init(*objectId,triOffset /* triOffset */,numPixels);
		populate_bake_data(bake_data,*objectId,pixelArray.data(),numPixels);

		if(worker.IsCancelled())
			return;

		worker.UpdateProgress(0.2f);

		m_cclSession->tile_manager.set_samples(m_cclSession->params.samples);
		m_cclSession->reset(const_cast<ccl::BufferParams&>(bufferParams), m_cclSession->params.samples);
		m_cclSession->update_scene();

		auto imgBuffer = uimg::ImageBuffer::Create(imgWidth,imgHeight,uimg::Format::RGBA_FLOAT);
		std::atomic<bool> running = true;
		//session->progress.set_update_callback(
		std::thread progressThread {[this,&worker,&running]() {
			while(running)
			{
				// Note: get_progress is technically not thread safe, but the worst we can expect is the progress to not be accurate once
				// in a while
				auto progress = m_cclSession->progress.get_progress();
				worker.UpdateProgress(0.2f +(0.95f -0.2f) *progress);
				std::this_thread::sleep_for(std::chrono::seconds{1});
			}
		}};
		auto r = m_cclScene->bake_manager->bake(m_cclScene->device,&m_cclScene->dscene,m_cclScene,m_cclSession->progress,shaderType,bake_pass_filter,bake_data,static_cast<float*>(imgBuffer->GetData()));
		running = false;
		progressThread.join();
		if(r == false)
		{
			worker.SetStatus(util::JobStatus::Failed,"Cycles baking has failed for an unknown reason!");
			return;
		}

		if(worker.IsCancelled())
			return;

		worker.UpdateProgress(0.95f);

		if(worker.IsCancelled())
			return;

		// Note: Denoising may not work well with baked images, since we can't supply any geometry information,
		// but the result is decent enough as long as the sample count is high.
		if(m_scene->ShouldDenoise())
		{
			/*if(m_renderMode == RenderMode::BakeDiffuseLighting)
			{
			auto lightmapInfo = m_lightmapTargetComponent.valid() ? m_lightmapTargetComponent->GetLightmapInfo() : nullptr;
			if(lightmapInfo)
			{
			auto originalResolution = lightmapInfo->atlasSize;
			// All of the lightmaps have to be denoised individually
			for(auto &rect : lightmapInfo->lightmapAtlas)
			{
			auto x = rect.x +lightmapInfo->borderSize;
			auto y = rect.y +lightmapInfo->borderSize;
			auto w = rect.w -lightmapInfo->borderSize *2;
			auto h = rect.h -lightmapInfo->borderSize *2;
			Vector2 offset {
			x /static_cast<float>(originalResolution),
			(originalResolution -y -h) /static_cast<float>(originalResolution) // Note: y is flipped!
			};
			Vector2 size {
			w /static_cast<float>(originalResolution),
			h /static_cast<float>(originalResolution)
			};

			DenoiseHDRImageArea(
			*imgBuffer,imgWidth,imgHeight,
			umath::clamp<float>(umath::round(offset.x *static_cast<float>(imgWidth)),0.f,imgWidth) +0.001f, // Add a small offset to make sure something like 2.9999 isn't truncated to 2
			umath::clamp<float>(umath::round(offset.y *static_cast<float>(imgHeight)),0.f,imgHeight) +0.001f,
			umath::clamp<float>(umath::round(size.x *static_cast<float>(imgWidth)),0.f,imgWidth) +0.001f,
			umath::clamp<float>(umath::round(size.y *static_cast<float>(imgHeight)),0.f,imgHeight) +0.001f
			);
			}
			}
			}
			else*/
			{
				// TODO: Check if denoise flag is set
				// Denoise the result. This has to be done before applying the margin! (Otherwise noise may flow into the margin)

				worker.SetResultMessage("Denoising...");
				DenoiseInfo denoiseInfo {};
				denoiseInfo.hdr = true;
				denoiseInfo.lightmap = bakeLightmaps;
				denoiseInfo.width = imgWidth;
				denoiseInfo.height = imgHeight;
				denoise(denoiseInfo,*imgBuffer,nullptr,nullptr,[this,&worker](float progress) -> bool {
					worker.UpdateProgress(0.95f +progress *0.2f);
					return !worker.IsCancelled();
				});
			}
		}

		if(worker.IsCancelled())
			return;

		// Applying color transform
		if(m_renderMode == Scene::RenderMode::BakeDiffuseLighting)
		{
			worker.SetResultMessage("Applying color transform...");
			auto &createInfo = m_scene->GetCreateInfo();
			if(createInfo.colorTransform.has_value())
			{
				std::string err;
				ColorTransformProcessorCreateInfo ctpCreateInfo {};
				ctpCreateInfo.config = createInfo.colorTransform->config;
				ctpCreateInfo.lookName = createInfo.colorTransform->lookName;
				m_colorTransformProcessor = create_color_transform_processor(ctpCreateInfo,err);
				if(m_colorTransformProcessor == nullptr)
					m_scene->HandleError("Unable to initialize color transform processor: " +err);
				else
					m_colorTransformProcessor->Apply(*imgBuffer,err,0.f /* exposure */,1.f /* gamma correction */);
			}
			if(worker.IsCancelled())
				return;
		}

		// if(m_renderMode == RenderMode::BakeDiffuseLighting)
		{
			worker.SetResultMessage("Baking margin...");

			imgBuffer->Convert(uimg::Format::RGBA_FLOAT);
			unirender::baking::ImBuf ibuf {};
			ibuf.x = imgWidth;
			ibuf.y = imgHeight;
			ibuf.rect = imgBuffer;

			// Apply margin
			// TODO: Margin only required for certain bake types?
			std::vector<uint8_t> mask_buffer {};
			mask_buffer.resize(numPixels);
			constexpr auto margin = 16u;
			RE_bake_mask_fill(pixelArray, numPixels, reinterpret_cast<char*>(mask_buffer.data()));
			RE_bake_margin(&ibuf, mask_buffer, margin);

			if(worker.IsCancelled())
				return;
		}

		ApplyPostProcessing(*imgBuffer,m_renderMode);

		if(worker.IsCancelled())
			return;

		auto exposure = m_scene->GetSceneInfo().exposure;
		if(exposure != 1.f)
		{
			for(auto &pxView : *imgBuffer)
			{
				for(auto channel : {uimg::Channel::Red,uimg::Channel::Green,uimg::Channel::Blue})
					pxView.SetValue(channel,pxView.GetFloatValue(channel) *exposure);
			}
		}

		if(umath::is_flag_set(m_scene->GetStateFlags(),Scene::StateFlags::OutputResultWithHDRColors) == false)
		{
			// Convert baked data to rgba8
			auto imgBufLDR = imgBuffer->Copy(uimg::Format::RGBA_LDR);
			auto numChannels = umath::to_integral(uimg::Channel::Count);
			auto itSrc = imgBuffer->begin();
			for(auto &pxViewDst : *imgBufLDR)
			{
				auto &pxViewSrc = *itSrc;
				for(auto i=decltype(numChannels){0u};i<numChannels;++i)
					pxViewDst.SetValue(static_cast<uimg::Channel>(i),baking::unit_float_to_uchar_clamp(pxViewSrc.GetFloatValue(static_cast<uimg::Channel>(i))));
				++itSrc;
			}
			imgBuffer = imgBufLDR;
			//auto f = FileManager::OpenFile<VFilePtrReal>("test_ao.png","wb");
			//uimg::save_image(f,*imgBuffer,uimg::ImageFormat::PNG);
			//f = nullptr;
		}
		else
		{
			// Image data is float data, but we only need 16 bits for our purposes
			auto imgBufHDR = imgBuffer->Copy(uimg::Format::RGBA_HDR);
			auto numChannels = umath::to_integral(uimg::Channel::Count);
			auto itSrc = imgBuffer->begin();
			for(auto &pxViewDst : *imgBufHDR)
			{
				auto &pxViewSrc = *itSrc;
				for(auto i=decltype(numChannels){0u};i<numChannels;++i)
					pxViewDst.SetValue(static_cast<uimg::Channel>(i),static_cast<uint16_t>(umath::float32_to_float16_glm(pxViewSrc.GetFloatValue(static_cast<uimg::Channel>(i)))));
				++itSrc;
			}
			imgBuffer = imgBufHDR;
			//auto f = FileManager::OpenFile<VFilePtrReal>("test_lightmap.png","wb");
			//uimg::save_image(f,*imgBuffer,uimg::ImageFormat::PNG);
			//f = nullptr;
		}

		if(worker.IsCancelled())
			return;

		GetResultImageBuffer(OUTPUT_COLOR) = imgBuffer;
		// m_cclSession->params.write_render_cb(static_cast<ccl::uchar*>(imgBuffer->GetData()),imgWidth,imgHeight,4 /* channels */);
		m_cclSession->params.write_render_cb = nullptr; // Make sure it's not called on destruction
		worker.SetStatus(util::JobStatus::Successful,"Baking has been completed successfully!");
		worker.UpdateProgress(1.f);
	});
#endif
}

void unirender::cycles::Renderer::FinalizeAndCloseCyclesScene()
{
	auto stateFlags = m_scene->GetStateFlags();
	if(m_cclSession && m_outputDriver && m_scene->IsRenderSceneMode(m_scene->GetRenderMode()) && umath::is_flag_set(m_stateFlags,StateFlags::RenderingStarted))
		GetResultImageBuffer(OUTPUT_COLOR) = GetOutputDriver()->GetImageBuffer(OUTPUT_COLOR);
	CloseCyclesScene();
}

void unirender::cycles::Renderer::CloseRenderScene() {CloseCyclesScene();}

void unirender::cycles::Renderer::FinalizeImage(uimg::ImageBuffer &imgBuf,StereoEye eyeStage)
{
	if(!m_scene->HasBakeTarget()) // Don't need to flip if we're baking
		imgBuf.Flip(true,true);
}

void unirender::cycles::Renderer::SetupRenderSettings(
	ccl::Scene &scene,ccl::Session &session,ccl::BufferParams &bufferParams,unirender::Scene::RenderMode renderMode,
	uint32_t maxTransparencyBounces
) const
{
	auto &sceneInfo = m_scene->GetSceneInfo();
	// Default parameters taken from Blender
	auto &integrator = *scene.integrator;
	auto &film = *scene.film;
	
	auto apiData = GetApiData();
	integrator.set_min_bounce(0);
	integrator.set_max_bounce(sceneInfo.maxBounces);
	integrator.set_max_diffuse_bounce(sceneInfo.maxDiffuseBounces);
	integrator.set_max_glossy_bounce(sceneInfo.maxGlossyBounces);
	integrator.set_max_transmission_bounce(sceneInfo.maxTransmissionBounces);
	integrator.set_max_volume_bounce(0);

	integrator.set_transparent_min_bounce(0);
	integrator.set_transparent_max_bounce(maxTransparencyBounces);

	integrator.set_volume_max_steps(1024);
	integrator.set_volume_step_rate(0.1);

	integrator.set_caustics_reflective(true);
	integrator.set_caustics_refractive(true);
	integrator.set_filter_glossy(0.f);
	integrator.set_seed(0);
	integrator.set_sampling_pattern(ccl::SamplingPattern::SAMPLING_PATTERN_SOBOL_BURLEY);

	auto useAdaptiveSampling = sceneInfo.useAdaptiveSampling;
	auto adaptiveSamplingThreshold = sceneInfo.adaptiveSamplingThreshold;
	auto adaptiveMinSamples = sceneInfo.adaptiveMinSamples;
	integrator.set_adaptive_threshold(sceneInfo.adaptiveSamplingThreshold);
	integrator.set_adaptive_min_samples(sceneInfo.adaptiveSamplingThreshold);

	apiData.GetFromPath("cycles/useAdaptiveSampling")(useAdaptiveSampling);
	if(useAdaptiveSampling)
	{
		apiData.GetFromPath("cycles/adaptiveSamplingThreshold")(adaptiveSamplingThreshold);
		apiData.GetFromPath("cycles/adaptiveMinSamples")(adaptiveMinSamples);
	}
	integrator.set_use_adaptive_sampling(useAdaptiveSampling);
	integrator.set_adaptive_threshold(adaptiveSamplingThreshold);
	integrator.set_adaptive_min_samples(adaptiveMinSamples);

#if 0
	// Debug values
	integrator.set_sample_clamp_direct(0.f);
	integrator.set_sample_clamp_indirect(0.f);
	integrator.set_motion_blur(false);
	//integrator.set_method(ccl::Integrator::Method::PATH);
	//integrator.set_sample_all_lights_direct(true);
	//integrator.set_sample_all_lights_indirect(true);
	integrator.set_light_sampling_threshold(0.f);

	//integrator.set_diffuse_samples(1);
	//integrator.set_glossy_samples(1);
	//integrator.set_transmission_samples(1);
	//integrator.set_ao_samples(1);
	//integrator.set_mesh_light_samples(1);
	//integrator.set_subsurface_samples(1);
	//integrator.set_volume_samples(1);

	integrator.set_ao_bounces(0);



  integrator.set_min_bounce(0);
  integrator.set_max_bounce(12);

  integrator.set_max_diffuse_bounce(4);
  integrator.set_max_glossy_bounce(4);
  integrator.set_max_transmission_bounce(12);
  integrator.set_max_volume_bounce(0);

  integrator.set_transparent_min_bounce(0);
  integrator.set_transparent_max_bounce(8);

  integrator.set_volume_max_steps(1024);
  integrator.set_volume_step_rate(1);

  integrator.set_caustics_reflective(true);
  integrator.set_caustics_refractive(true);
  integrator.set_filter_glossy(1);

  integrator.set_sample_clamp_direct(0);
  integrator.set_sample_clamp_indirect(10);
  integrator.set_motion_blur(false);

  integrator.set_light_sampling_threshold(0.00999999978);

  integrator.set_sampling_pattern(ccl::SAMPLING_PATTERN_PMJ);

    integrator.set_use_adaptive_sampling(true);
    integrator.set_adaptive_threshold(0.00999999978);
    integrator.set_adaptive_min_samples(0);

  integrator.set_scrambling_distance(1);

      integrator.set_ao_bounces(0);

  integrator.set_use_denoise(true);

  /* Only update denoiser parameters if the denoiser is actually used. This allows to tweak
   * denoiser parameters before enabling it without render resetting on every change. The downside
   * is that the interface and the integrator are technically out of sync. */

    integrator.set_denoiser_type(ccl::DENOISER_OPENIMAGEDENOISE);
    integrator.set_denoise_start_sample(0);
    integrator.set_use_denoise_pass_albedo(true);
    integrator.set_use_denoise_pass_normal(true);
    integrator.set_denoiser_prefilter(ccl::DENOISER_PREFILTER_ACCURATE);
#endif
	integrator.tag_modified();

	// Film
	film.set_exposure(1.f);
	film.set_filter_type(ccl::FilterType::FILTER_GAUSSIAN);
	film.set_filter_width(1.5f);
	if(renderMode == unirender::Scene::RenderMode::RenderImage)
	{
		film.set_mist_start(5.f);
		film.set_mist_depth(25.f);
		film.set_mist_falloff(2.f);
	}
	film.tag_modified();

	film.set_cryptomatte_depth(3);
	film.set_cryptomatte_passes(ccl::CRYPT_NONE);

#if 0
	// Debug values
	film.set_exposure(0.800000012);
	film.set_filter_type(ccl::FILTER_BLACKMAN_HARRIS);
	film.set_filter_width(1.50000000);
	film.set_mist_start(5.00000000);
	film.set_mist_depth(25.0000000);
	film.set_mist_falloff(2.0f);
	film.set_use_approximate_shadow_catcher(true);
#endif

	session.params.pixel_size = 1;
	session.params.threads = 0;
	session.params.use_profiling = false;
	session.params.shadingsystem = ccl::ShadingSystem::SHADINGSYSTEM_SVM;

	for(auto &pair : m_outputs)
	{
		if(pair.first == OUTPUT_COLOR)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_COLOR});
			pass->set_type(ccl::PASS_COMBINED);
			break;
		}
		else if(pair.first == OUTPUT_DIFFUSE)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_DIFFUSE});
			pass->set_type(ccl::PASS_DIFFUSE);
			pass->set_include_albedo(false);
		}
		else if(pair.first == OUTPUT_DIFFUSE_DIRECT)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_DIFFUSE_DIRECT});
			pass->set_type(ccl::PASS_DIFFUSE_DIRECT);
			pass->set_include_albedo(false);
		}
		else if(pair.first == OUTPUT_DIFFUSE_INDIRECT)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_DIFFUSE_INDIRECT});
			pass->set_type(ccl::PASS_DIFFUSE_INDIRECT);
			pass->set_include_albedo(false);
		}
		else if(pair.first == OUTPUT_ALBEDO)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_ALBEDO});
			if(m_scene->GetRenderMode() == unirender::Scene::RenderMode::SceneAlbedo)
				pass->set_type(ccl::PASS_DIFFUSE_COLOR);
			else
				pass->set_type(ccl::PASS_DENOISING_ALBEDO);
		}
		else if(pair.first == OUTPUT_NORMAL)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_NORMAL});
			if(m_scene->GetRenderMode() == unirender::Scene::RenderMode::SceneNormals)
				pass->set_type(ccl::PASS_NORMAL);
			else
				pass->set_type(ccl::PASS_DENOISING_NORMAL);
		}
		else if(pair.first == OUTPUT_DEPTH)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_DEPTH});
			if(m_scene->GetRenderMode() == unirender::Scene::RenderMode::SceneDepth)
				pass->set_type(ccl::PASS_DEPTH);
			else
				pass->set_type(ccl::PASS_DENOISING_DEPTH);
		}
		else if(pair.first == OUTPUT_AO)
		{
			auto *pass = scene.create_node<ccl::Pass>();
			pass->set_name(ccl::ustring{OUTPUT_AO});
			pass->set_type(ccl::PASS_AO);
			pass->set_include_albedo(false);
		}
	}

#if 0
	ccl::vector<ccl::BufferPass> passes;
	auto displayPass = ccl::PassType::PASS_DIFFUSE_COLOR;
	switch(renderMode)
	{
	case unirender::Scene::RenderMode::SceneAlbedo:
		// Note: PASS_DIFFUSE_COLOR would probably make more sense but does not seem to work
		// (just creates a black output).
		ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined");
		ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
		displayPass = ccl::PassType::PASS_COMBINED;
		break;
	case unirender::Scene::RenderMode::SceneNormals:
		ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined");
		ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
		displayPass = ccl::PassType::PASS_COMBINED;
		break;
	case unirender::Scene::RenderMode::SceneDepth:
		ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined"); // TODO: Why do we need this?
		ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
		displayPass = ccl::PassType::PASS_COMBINED;
		break;
	case unirender::Scene::RenderMode::RenderImage:
		ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined");
		ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
		displayPass = ccl::PassType::PASS_COMBINED;
		break;
	case unirender::Scene::RenderMode::BakeAmbientOcclusion:
		ccl::Pass::add(ccl::PassType::PASS_AO,passes,"ao");
		ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
		displayPass = ccl::PassType::PASS_AO;
		break;
	case unirender::Scene::RenderMode::BakeDiffuseLighting:
		//ccl::Pass::add(ccl::PassType::PASS_DIFFUSE_DIRECT,passes,"diffuse_direct");
		//ccl::Pass::add(ccl::PassType::PASS_DIFFUSE_INDIRECT,passes,"diffuse_indirect");
		ccl::Pass::add(ccl::PassType::PASS_COMBINED,passes,"combined");
		ccl::Pass::add(ccl::PassType::PASS_DEPTH,passes,"depth");
		displayPass = ccl::PassType::PASS_COMBINED; // TODO: Is this correct?
		break;
	}
	bufferParams.passes = passes;
#endif

	if(sceneInfo.motionBlurStrength > 0.f)
	{
		// ccl::Pass::add(ccl::PassType::PASS_MOTION,passes);
		scene.integrator->set_motion_blur(true);
	}

	film.set_pass_alpha_threshold(0.5);
	//film.set_tag_passes_update(&scene, passes);
	//film.set_display_pass(displayPass);
	//film.tag_update(&scene);
	//scene.integrator->tag_update(&scene);

	// Camera
	/*auto &cam = *scene.camera;
	cam.shuttertime	= 0.500000000;
	cam.motion_position=	ccl::Camera::MotionPosition::MOTION_POSITION_CENTER;
	cam.shutter_table_offset=	18446744073709551615;
	cam.rolling_shutter_type=	ccl::Camera::RollingShutterType::ROLLING_SHUTTER_NONE;
	cam.rolling_shutter_duration=	0.100000001	;
	cam.focaldistance=	2.49260306	;
	cam.aperturesize=	0.00625000009	;
	cam.blades=	0;
	cam.bladesrotation=	0.000000000	;
	cam.type=	ccl::CAMERA_PERSPECTIVE ;
	cam.fov=	0.503379941	;
	cam.panorama_type=	ccl::PANORAMA_FISHEYE_EQUISOLID ;
	cam.fisheye_fov=	3.14159274	;
	cam.fisheye_lens=	10.5000000	;
	cam.latitude_min=	-1.57079637	;
	cam.latitude_max=	1.57079637	;
	cam.longitude_min=	-3.14159274	;
	cam.longitude_max=	3.14159274	;
	cam.stereo_eye=	ccl::Camera::STEREO_NONE ;
	cam.use_spherical_stereo=	false	;
	cam.interocular_distance=	0.0649999976	;
	cam.convergence_distance=	1.94999993	;
	cam.use_pole_merge	=false	;
	cam.pole_merge_angle_from=	1.04719758	;
	cam.pole_merge_angle_to=	1.30899692	;
	cam.aperture_ratio=	1.00000000	;
	cam.sensorwidth=	0.0359999985	;
	cam.sensorheight=	0.0240000002	;
	cam.nearclip=	0.100000001	;
	cam.farclip	=100.000000	;
	cam.width	=3840	;
	cam.height=	2160	;
	cam.resolution=	1	;

	cam.viewplane.left=	-1.77777779	;
	cam.viewplane.right=	1.77777779	;
	cam.viewplane.bottom=	-1.00000000	;
	cam.viewplane.top=	1.00000000	;


	cam.full_width=	3840	;
	cam.full_height=	2160	;
	cam.offscreen_dicing_scale	=4.00000000	;
	cam.border.left = 0.f;
	cam.border.right = 1.f;
	cam.border.bottom = 0.f;
	cam.border.top = 1.f;
	cam.viewport_camera_border .left = 0.f;
	cam.viewport_camera_border.right = 1.f;
	cam.viewport_camera_border.bottom = 0.f;
	cam.viewport_camera_border.top = 1.f;

	cam.matrix.x.x=	1.00000000	;
	cam.matrix.x.y=	1.63195708e-07	;
	cam.matrix.x.z=	3.42843151e-07	;
	cam.matrix.x.w=	17.5277958	;

	cam.matrix.y.x=	-3.47716451e-07	;
	cam.matrix.y.y	=0.0308625121	;
	cam.matrix.y.z=	0.999523640	;
	cam.matrix.y.w=	-2.77792454	;

	cam.matrix.z.x=	-1.52536970e-07	;
	cam.matrix.z.y=	0.999523640	;
	cam.matrix.z.z=	-0.0308625121	;
	cam.matrix.z.w=	0.846632719	;

	cam.use_perspective_motion=	false	;
	cam.fov_pre=	0.503379941	;
	cam.fov_post	=0.503379941	;

	cam.tag_update();*/
}
