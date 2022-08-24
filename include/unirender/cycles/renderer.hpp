/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#ifndef __UNIRENDER_CYCLES_SCENE_HPP__
#define __UNIRENDER_CYCLES_SCENE_HPP__

#include <util_raytracing/object.hpp>
#include <util_raytracing/renderer.hpp>
#include <util_raytracing/scene.hpp>
#include <sharedutils/util_baking.hpp>
#include <session/session.h>
#include <cinttypes>
#include <atomic>

namespace unirender
{
	class Scene; namespace cycles {class Renderer;};
};
namespace unirender::cycles
{
	void compute_tangents(ccl::Mesh *mesh,bool need_sign,bool active_render);
	class DisplayDriver;
	class OutputDriver;
	class Renderer
		: public unirender::Renderer
	{
	public:
		static std::shared_ptr<Renderer> Create(const unirender::Scene &scene,Flags flags);
		static constexpr ccl::AttributeStandard ALPHA_ATTRIBUTE_TYPE = ccl::AttributeStandard::ATTR_STD_POINTINESS;

		enum class StateFlags : uint32_t
		{
			None = 0u,
			SkyInitialized = 1u,
			RenderingStarted = SkyInitialized<<1u,
			ProgressiveRefine = RenderingStarted<<1u,
			NativeDenoising = ProgressiveRefine<<1u,
			SessionWasStarted = NativeDenoising<<1u,
			ReloadSessionScheduled = SessionWasStarted<<1u
		};

		static Vector3 ToPragmaPosition(const ccl::float3 &pos);
		static ccl::float3 ToCyclesVector(const Vector3 &v);
		static ccl::float3 ToCyclesPosition(const Vector3 &pos);
		static ccl::float3 ToCyclesNormal(const Vector3 &n);
		static ccl::float2 ToCyclesUV(const Vector2 &uv);
		static ccl::Transform ToCyclesTransform(const umath::ScaledTransform &t,bool applyRotOffset=false);
		static float ToCyclesLength(float len);

		virtual ~Renderer() override;
		virtual void Wait() override;
		virtual void Start() override {} // TODO: Remove
		virtual float GetProgress() const override;
		virtual bool Stop() override;
		virtual bool Pause() override;
		virtual bool Resume() override;
		virtual bool Suspend() override;
		virtual bool Export(const std::string &path) override;
		virtual void Reset() override;
		virtual void Restart() override;
		virtual bool ShouldUseProgressiveFloatFormat() const override {return false;}
		virtual std::optional<std::string> SaveRenderPreview(const std::string &path,std::string &outErr) const override;
		virtual util::ParallelJob<uimg::ImageLayerSet> StartRender() override;

		virtual bool BeginSceneEdit() override;
		virtual bool EndSceneEdit() override;
		virtual bool SyncEditedActor(const util::Uuid &uuid) override;
		virtual bool AddLiveActor(unirender::WorldObject &actor) override;

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

		DisplayDriver *GetDisplayDriver() {return m_displayDriver;}
		OutputDriver *GetOutputDriver() {return m_outputDriver;}
		//static void UpdateRenderTile(TileManager &tileManager,const ccl::RenderTile &tile,bool param);
		//static void WriteRenderTile(TileManager &tileManager,const ccl::RenderTile &tile);

		// For internal use only
		void SetStereoscopicEye(StereoEye eye);
	private:
		Renderer(const Scene &scene,Flags flags);

		static ccl::ShaderOutput *FindShaderNodeOutput(ccl::ShaderNode &node,const std::string &output);
		static ccl::ShaderNode *FindShaderNode(ccl::ShaderGraph &graph,const std::string &nodeName);
		static ccl::ShaderNode *FindShaderNode(ccl::ShaderGraph &graph,const OpenImageIO_v2_1::ustring &name);

		virtual void SetCancelled(const std::string &msg="Cancelled by application.") override;
		bool InitializeBakingData();
		void FinalizeAndCloseCyclesScene();
		void CloseCyclesScene();
		void ApplyPostProcessing(uimg::ImageBuffer &imgBuffer,unirender::Scene::RenderMode renderMode);
		void InitializeAlbedoPass(bool reloadShaders);
		void InitializeNormalPass(bool reloadShaders);
		void InitializePassShaders(const std::function<std::shared_ptr<GroupNodeDesc>(const Shader&)> &fGetPassDesc);
		void AddSkybox(const std::string &texture);
		virtual util::EventReply HandleRenderStage(RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye eyeStage,unirender::Renderer::RenderStageResult *optResult=nullptr) override;
		void WaitForRenderStage(RenderWorker &worker,float baseProgress,float progressMultiplier,const std::function<unirender::Renderer::RenderStageResult()> &fOnComplete);
		void StartTextureBaking(RenderWorker &worker);
		virtual void PrepareCyclesSceneForRendering() override;
		virtual bool UpdateStereoEye(unirender::RenderWorker &worker,unirender::Renderer::ImageRenderStage stage,StereoEye &eyeStage) override;
		virtual void CloseRenderScene() override;
		virtual void FinalizeImage(uimg::ImageBuffer &imgBuf,StereoEye eyeStage) override;
		void InitStereoEye(StereoEye eyeStage);
		void ReloadProgressiveRender(bool clearExposure=true,bool waitForPreviousCompletion=false);
		int GetTileSize() const;

		void SetupRenderSettings(
			ccl::Scene &scene,ccl::Session &session,ccl::BufferParams &bufferParams,unirender::Scene::RenderMode renderMode,
			uint32_t maxTransparencyBounces
		) const;

		void AddDebugSky();
		ccl::Mesh *AddDebugMesh();
		ccl::Object *AddDebugObject();
		ccl::Shader *AddDebugShader();
		void AddDebugLight();
		void InitializeDebugScene(const std::string &fileName,const std::vector<std::string> &xmlFileNames);
		void PopulateDebugScene();

		ccl::SessionParams GetSessionParameters(const unirender::Scene &scene,const ccl::DeviceInfo &devInfo) const;
		ccl::BufferParams GetBufferParameters() const;
		void SyncLight(unirender::Scene &scene,const unirender::Light &light,bool update=false);
		void SyncCamera(const unirender::Camera &cam,bool update=false);
		void SyncObject(const unirender::Object &obj);
		void SyncMesh(const unirender::Mesh &mesh);
		void InitializeSession(unirender::Scene &scene,const ccl::DeviceInfo &devInfo);
		std::optional<ccl::DeviceInfo> InitializeDevice(const unirender::Scene &scene,std::string &outErr);
		bool Initialize(unirender::Scene &scene,std::string &outErr);
		unirender::Scene::DeviceType m_deviceType = unirender::Scene::DeviceType::CPU;
		std::unique_ptr<ccl::Session> m_cclSession = nullptr;
		DisplayDriver *m_displayDriver = nullptr;
		OutputDriver *m_outputDriver = nullptr;
		ccl::Scene *m_cclScene = nullptr;
		std::vector<std::shared_ptr<CCLShader>> m_cclShaders = {};
		std::unordered_map<const GroupNodeDesc*,size_t> m_shaderCache {};
		struct CclObjectInfo
		{
			ccl::Object *object = nullptr;
			umath::ScaledTransform lastUpdatePose {};
		};
		std::unordered_map<const Object*,CclObjectInfo> m_objectToCclObject;
		std::unordered_map<std::string,const Object*> m_uuidToObject;
		std::unordered_map<const Mesh*,ccl::Mesh*> m_meshToCcclMesh;
		std::unordered_map<ccl::Mesh*,const Mesh*> m_cclMeshToMesh;
		std::unordered_map<const Light*,ccl::Light*> m_lightToCclLight;
		std::unordered_map<const unirender::Light*,std::shared_ptr<CCLShader>> m_lightToShader;
		std::atomic<uint32_t> m_restartState = 0;
		StateFlags m_stateFlags = StateFlags::None;
		std::mutex m_cancelMutex;
		bool m_cancelled = false;

		std::unique_ptr<util::baking::BakeDataView> m_bakeData = nullptr;
		std::vector<util::baking::BakePixel> m_bakePixels;

		ccl::SessionParams m_sessionParams;
		ccl::BufferParams m_bufferParams;

		Scene::RenderMode m_renderMode = Scene::RenderMode::RenderImage;
	};
};
REGISTER_BASIC_BITWISE_OPERATORS(unirender::cycles::Renderer::StateFlags)

#endif
