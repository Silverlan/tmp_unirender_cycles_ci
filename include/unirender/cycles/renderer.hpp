/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2020 Florian Weischer
*/

#ifndef __UNIRENDER_CYCLES_SCENE_HPP__
#define __UNIRENDER_CYCLES_SCENE_HPP__

#include "../object.hpp"
#include "../renderer.hpp"
#include "../scene.hpp"
#include <render/session.h>
#include <cinttypes>
#include <atomic>

namespace unirender {class Scene; namespace cycles {class Renderer;};};
namespace unirender::cycles
{
	void compute_tangents(ccl::Mesh *mesh,bool need_sign,bool active_render);
	class DLLRTUTIL Renderer
		: public unirender::Renderer
	{
	public:
		static std::shared_ptr<Renderer> Create(const unirender::Scene &scene);
		static constexpr ccl::AttributeStandard ALPHA_ATTRIBUTE_TYPE = ccl::AttributeStandard::ATTR_STD_POINTINESS;

		virtual ~Renderer() override;
		virtual void Wait() override;
		virtual void Start() override {} // TODO: Remove
		virtual float GetProgress() const override;
		virtual void Reset() override;
		virtual void Restart() override;
		virtual util::ParallelJob<std::shared_ptr<uimg::ImageBuffer>> StartRender() override;

		ccl::Object *FindCclObject(const Object &obj);
		const ccl::Object *FindCclObject(const Object &obj) const {return const_cast<Renderer*>(this)->FindCclObject(obj);}
		ccl::Mesh *FindCclMesh(const Mesh &mesh);
		const ccl::Mesh *FindCclMesh(const Mesh &mesh) const {return const_cast<Renderer*>(this)->FindCclMesh(mesh);}
		ccl::Light *FindCclLight(const Light &light);
		const ccl::Light *FindCclLight(const Light &light) const {return const_cast<Renderer*>(this)->FindCclLight(light);}

		ccl::Scene *operator->() {return m_cclScene;}
		const ccl::Scene *operator->() const {return const_cast<Renderer*>(this)->operator->();}
		ccl::Scene &operator*() {return *operator->();}
		const ccl::Scene &operator*() const {return const_cast<Renderer*>(this)->operator*();}

		std::optional<uint32_t> FindCCLObjectId(const ccl::Object &o) const;
		ccl::Session *GetCclSession() {return m_cclSession.get();}
		const ccl::Session *GetCclSession() const {return const_cast<Renderer*>(this)->GetCclSession();}
		ccl::Scene *GetCclScene() {return m_cclScene;}
		const ccl::Scene *GetCclScene() const {return const_cast<Renderer*>(this)->GetCclScene();}

		std::shared_ptr<CCLShader> GetCachedShader(const GroupNodeDesc &desc) const;
		void AddShader(CCLShader &shader,const GroupNodeDesc *optDesc=nullptr);

		// For internal use only
		void SetStereoscopicEye(StereoEye eye);
	private:
		Renderer(const Scene &scene);
		Object *FindObject(const std::string &objectName) const;
		virtual void SetCancelled(const std::string &msg="Cancelled by application.") override;
		void FinalizeAndCloseCyclesScene();
		void CloseCyclesScene();
		void ApplyPostProcessing(uimg::ImageBuffer &imgBuffer,unirender::Scene::RenderMode renderMode);
		std::shared_ptr<uimg::ImageBuffer> FinalizeCyclesScene();
		void InitializeAlbedoPass(bool reloadShaders);
		void InitializeNormalPass(bool reloadShaders);
		void InitializePassShaders(const std::function<std::shared_ptr<GroupNodeDesc>(const Shader&)> &fGetPassDesc);
		void AddSkybox(const std::string &texture);
		unirender::Renderer::RenderStageResult StartNextRenderImageStage(RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye eyeStage);
		void WaitForRenderStage(RenderWorker &worker,float baseProgress,float progressMultiplier,const std::function<unirender::Renderer::RenderStageResult()> &fOnComplete);
		void StartTextureBaking(RenderWorker &worker);
		virtual void PrepareCyclesSceneForRendering() override;
		bool UpdateStereo(unirender::RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye &eyeStage);
		void ReloadProgressiveRender(bool clearExposure=true,bool waitForPreviousCompletion=false);
		Vector2i GetTileSize() const;

		void SetupRenderSettings(
			ccl::Scene &scene,ccl::Session &session,ccl::BufferParams &bufferParams,unirender::Scene::RenderMode renderMode,
			uint32_t maxTransparencyBounces
		) const;

		ccl::SessionParams GetSessionParameters(const unirender::Scene &scene,const ccl::DeviceInfo &devInfo) const;
		ccl::BufferParams GetBufferParameters() const;
		void SyncLight(unirender::Scene &scene,const unirender::Light &light);
		void SyncCamera(const unirender::Camera &cam);
		void SyncObject(const unirender::Object &obj);
		void SyncMesh(const unirender::Mesh &mesh);
		void InitializeSession(unirender::Scene &scene,const ccl::DeviceInfo &devInfo);
		std::optional<ccl::DeviceInfo> InitializeDevice(const unirender::Scene &scene);
		bool Initialize(unirender::Scene &scene);
		unirender::Scene::DeviceType m_deviceType = unirender::Scene::DeviceType::CPU;
		std::unique_ptr<ccl::Session> m_cclSession = nullptr;
		ccl::Scene *m_cclScene = nullptr;
		std::vector<std::shared_ptr<CCLShader>> m_cclShaders = {};
		std::unordered_map<const GroupNodeDesc*,size_t> m_shaderCache {};
		std::unordered_map<const Object*,ccl::Object*> m_objectToCclObject;
		std::unordered_map<const Mesh*,ccl::Mesh*> m_meshToCcclMesh;
		std::unordered_map<const Light*,ccl::Light*> m_lightToCclLight;
		std::atomic<uint32_t> m_restartState = 0;
		bool m_skyInitialized = false;
		bool m_renderingStarted = false;
		bool m_progressiveRefine = false;
		Scene::RenderMode m_renderMode = Scene::RenderMode::RenderImage;
	};
};

#endif
