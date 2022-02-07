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
#include <util/path.h>
#define ENABLE_CYCLES_LOGGING
#ifdef ENABLE_CYCLES_LOGGING
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#endif
#include <util_image_buffer.hpp>

#ifdef _WIN32
#include <Shlobj.h>
#endif

#pragma optimize("",off)

static std::optional<std::string> KERNEL_PATH {};
void unirender::Scene::SetKernelPath(const std::string &kernelPath) {KERNEL_PATH = kernelPath;}
int cycles_standalone_test(int argc, const char **argv,bool initPaths);
static void init_cycles()
{
	static auto isInitialized = false;
	if(isInitialized)
		return;
	isInitialized = true;

	std::string cyclesPath;
	if(KERNEL_PATH.has_value())
		cyclesPath = *KERNEL_PATH;
	else
		cyclesPath = util::get_program_path();
	
	for(auto &c : cyclesPath)
	{
		if(c == '\\')
			c = '/';
	}
	auto kernelPath = cyclesPath;
	ccl::path_init(kernelPath,kernelPath);

	util::set_env_variable("CYCLES_KERNEL_PATH",kernelPath);
	util::set_env_variable("CYCLES_SHADER_PATH",kernelPath);

	std::optional<std::string> optixPath {};
#ifdef _WIN32
	TCHAR szPath[MAX_PATH];
	if(SUCCEEDED(SHGetFolderPath(NULL, CSIDL_COMMON_APPDATA, NULL, 0, szPath)))
	{
		std::string path = szPath;
		path += "/NVIDIA Corporation/";
		std::vector<std::string> dirs;
		FileManager::FindSystemFiles((path +"OptiX SDK*").c_str(),nullptr,&dirs);
		if(!dirs.empty())
		{
			std::sort(dirs.begin(),dirs.end());
			auto dir = dirs.back(); // Presumably the latest version of the SDK
			optixPath = path +dir;
			for(auto &c : *optixPath)
			{
				if(c == '\\')
					c = '/';
			}
		}
	}
#endif

	if(optixPath.has_value())
	{
		std::cout<<"Found Optix SDK: "<<*optixPath<<std::endl;
		util::set_env_variable("OPTIX_ROOT_DIR",*optixPath);
		// Note: The above should work, but for some reason it doesn't in some cases?
		// We'll use putenv as well, just to be sure.
		putenv(("OPTIX_ROOT_DIR=" +*optixPath).c_str());
	}
	else
		std::cout<<"Could not find Optix SDK! Dynamic Optix kernel building will be disabled!"<<std::endl;
#ifdef ENABLE_CYCLES_LOGGING
	// ccl::util_logging_init("util_raytracing");
	// ccl::util_logging_verbosity_set(2);
	// ccl::util_logging_start();
	google::InitGoogleLogging("util_raytracing");
	google::SetLogDestination(google::GLOG_INFO,(cyclesPath +"/log/info.log").c_str());
	google::SetLogDestination(google::GLOG_WARNING,(cyclesPath +"/log/warning.log").c_str());
	google::SetLogDestination(google::GLOG_ERROR,(cyclesPath +"/log/error.log").c_str());
	google::SetLogDestination(google::GLOG_FATAL,(cyclesPath +"/log/fatal.log").c_str());
	FLAGS_log_dir = cyclesPath +"/log";
	//FLAGS_logtostderr = true;
	//FLAGS_alsologtostderr = true; // Doesn't seem to work properly?
	//FLAGS_stderrthreshold = google::GLOG_WARNING|google::GLOG_ERROR|google::GLOG_INFO|google::GLOG_FATAL;
	//FLAGS_v = 5; // Setting the log level any other way doesn't seem to work properly
	// LOG(INFO) << "Info Test 1";
	// google::LogAtLevel(google::GLOG_INFO,"Info test");
	// google::LogAtLevel(google::GLOG_WARNING,"Warning test");
#endif
}

static bool is_device_type_available(ccl::DeviceType type)
{
	using namespace ccl;
	return ccl::Device::available_devices(DEVICE_MASK(type)).empty() == false;
}

ccl::float3 unirender::cycles::Renderer::ToCyclesVector(const Vector3 &v)
{
	return ccl::float3{v.x,v.y,v.z};
}

Vector3 unirender::cycles::Renderer::ToPragmaPosition(const ccl::float3 &pos)
{
	auto scale = util::pragma::units_to_metres(1.f);
	Vector3 prPos {pos.x,pos.z,-pos.y};
	prPos /= scale;
	return prPos;
}

ccl::float3 unirender::cycles::Renderer::ToCyclesPosition(const Vector3 &pos)
{
	auto scale = util::pragma::units_to_metres(1.f);
#ifdef ENABLE_TEST_AMBIENT_OCCLUSION
	ccl::float3 cpos {pos.x,-pos.z,pos.y};
#else
	ccl::float3 cpos {-pos.x,pos.y,pos.z};
#endif
	cpos *= scale;
	return cpos;
}

ccl::float3 unirender::cycles::Renderer::ToCyclesNormal(const Vector3 &n)
{
#ifdef ENABLE_TEST_AMBIENT_OCCLUSION
	return ccl::float3{n.x,-n.z,n.y};
#else
	return ccl::float3{-n.x,n.y,n.z};
#endif
}

ccl::float2 unirender::cycles::Renderer::ToCyclesUV(const Vector2 &uv)
{
	return ccl::float2{uv.x,1.f -uv.y};
}

ccl::Transform unirender::cycles::Renderer::ToCyclesTransform(const umath::ScaledTransform &t,bool applyRotOffset)
{
	Vector3 axis;
	float angle;
	uquat::to_axis_angle(t.GetRotation(),axis,angle);
	auto cclT = ccl::transform_identity();
	cclT = cclT *ccl::transform_rotate(angle,ToCyclesNormal(axis));
	if(applyRotOffset)
		cclT = cclT *ccl::transform_rotate(umath::deg_to_rad(90.f),ccl::float3{1.f,0.f,0.f});
	cclT = ccl::transform_translate(ToCyclesPosition(t.GetOrigin())) *cclT;
	cclT = cclT *ccl::transform_scale(ToCyclesVector(t.GetScale()));
	return cclT;
}

float unirender::cycles::Renderer::ToCyclesLength(float len)
{
	auto scale = util::pragma::units_to_metres(1.f);
	return len *scale;
}

std::shared_ptr<unirender::cycles::Renderer> unirender::cycles::Renderer::Create(const unirender::Scene &scene,Flags flags)
{
	auto renderer = std::shared_ptr<Renderer>{new Renderer{scene}};
	renderer->m_renderMode = scene.GetRenderMode();

	auto &createInfo = scene.GetCreateInfo();
	umath::set_flag(renderer->m_stateFlags,StateFlags::ProgressiveRefine,createInfo.progressiveRefine);
	auto &sceneInfo = scene.GetSceneInfo();
	const_cast<Scene::SceneInfo&>(sceneInfo).exposure = createInfo.exposure; // TODO: This doesn't belong here!
	return renderer;
}

unirender::cycles::Renderer::Renderer(const Scene &scene)
	: unirender::Renderer{scene}
{}
unirender::cycles::Renderer::~Renderer()
{
	FinalizeAndCloseCyclesScene();
#ifdef ENABLE_CYCLES_LOGGING
	google::FlushLogFiles(google::GLOG_INFO);
	google::FlushLogFiles(google::GLOG_WARNING);
	google::FlushLogFiles(google::GLOG_ERROR);
	google::FlushLogFiles(google::GLOG_FATAL);
#endif
}

ccl::SessionParams unirender::cycles::Renderer::GetSessionParameters(const unirender::Scene &scene,const ccl::DeviceInfo &devInfo) const
{
	auto &createInfo = scene.GetCreateInfo();
	ccl::SessionParams sessionParams {};
	sessionParams.shadingsystem = ccl::SHADINGSYSTEM_SVM;
	sessionParams.device = devInfo;
	sessionParams.background = true;
	sessionParams.use_auto_tile = false;

	switch(m_deviceType)
	{
	case unirender::Scene::DeviceType::GPU:
		sessionParams.tile_size = 256;
		break;
	default:
		sessionParams.tile_size = 16;
		break;
	}

	if(createInfo.samples.has_value())
		sessionParams.samples = *createInfo.samples;
	else
	{
		switch(m_renderMode)
		{
		case unirender::Scene::RenderMode::BakeAmbientOcclusion:
		case unirender::Scene::RenderMode::BakeNormals:
		case unirender::Scene::RenderMode::BakeDiffuseLighting:
			sessionParams.samples = 1'225u;
			break;
		default:
			sessionParams.samples = 1'024u;
			break;
		}
	}

	if(umath::is_flag_set(m_stateFlags,StateFlags::ProgressiveRefine))
		sessionParams.samples = 50'000;

#ifdef ENABLE_TEST_AMBIENT_OCCLUSION
	if(unirender::Scene::IsRenderSceneMode(m_renderMode) == false)
	{
		//sessionParams.background = true;
		/*sessionParams.progressive_refine = false;
		sessionParams.progressive = false;
		sessionParams.experimental = false;
		sessionParams.tile_size = {256,256};
		sessionParams.tile_order = ccl::TileOrder::TILE_BOTTOM_TO_TOP;
		sessionParams.start_resolution = 2147483647;
		sessionParams.pixel_size = 1;
		sessionParams.threads = 0;
		sessionParams.use_profiling = false;
		sessionParams.display_buffer_linear = true;
		sessionParams.run_denoising = false;
		sessionParams.write_denoising_passes = false;
		sessionParams.full_denoising = false;
		sessionParams.progressive_update_timeout = 1.0000000000000000;
		sessionParams.shadingsystem = ccl::SHADINGSYSTEM_SVM;*/
	}
#endif
	return sessionParams;
}

// Hip is not yet fully implemented in Cycles X, so it's currently disabled (state: 22-02-03)
// #define ENABLE_AMD_HIP
std::optional<ccl::DeviceInfo> unirender::cycles::Renderer::InitializeDevice(const unirender::Scene &scene,std::string &outErr)
{
	init_cycles();

	auto cclDeviceType = ccl::DeviceType::DEVICE_CPU;
	constexpr std::array<ccl::DeviceType,4> gpuDeviceTypes = {
		// Order represents order of preference!
		ccl::DeviceType::DEVICE_OPTIX,
		ccl::DeviceType::DEVICE_CUDA,
#ifdef ENABLE_AMD_HIP
		ccl::DeviceType::DEVICE_HIP,
#endif
		ccl::DeviceType::DEVICE_MULTI // TODO: What's this one exactly?
	};
	auto devices = ccl::DeviceTypeMask::DEVICE_MASK_CUDA | ccl::DeviceTypeMask::DEVICE_MASK_OPTIX | ccl::DeviceTypeMask::DEVICE_MASK_CPU
#ifdef ENABLE_AMD_HIP
		 | ccl::DeviceTypeMask::DEVICE_MASK_HIP
#endif
		;
	
	auto &createInfo = scene.GetCreateInfo();

	auto apiData = GetApiData();
	std::string deviceOverride;
	apiData.GetFromPath("cycles/device")(deviceOverride);
	if(!deviceOverride.empty())
	{
		auto dev = ccl::Device::type_from_string(deviceOverride.c_str());
		if(dev == ccl::DeviceType::DEVICE_NONE)
		{
			std::string availableTypes;
			for(auto devType : ccl::Device::available_types())
			{
				if(!availableTypes.empty())
					availableTypes += ", ";
				availableTypes += ccl::Device::string_from_type(devType);
			}
			outErr = "Unknown or unsupported device '" +deviceOverride +"'! Available devices are: " +availableTypes +".";
			return {};
		}
		cclDeviceType = dev;
	}
	else
	{
		auto enableOptix = true;
		apiData.GetFromPath("cycles/enableOptix")(enableOptix); // Optix doesn't work properly on some devices, so we need an option to disable it
		switch(createInfo.deviceType)
		{
		case unirender::Scene::DeviceType::GPU:
		{
			for(auto devType : gpuDeviceTypes)
			{
				if(is_device_type_available(devType) && (devType != ccl::DeviceType::DEVICE_OPTIX || enableOptix))
				{
					cclDeviceType = devType;
					goto endLoop;
				}
			}
			// No break is intended!
		}
		case unirender::Scene::DeviceType::CPU:
			cclDeviceType = ccl::DeviceType::DEVICE_CPU;
			break;
		}
		static_assert(umath::to_integral(unirender::Scene::DeviceType::Count) == 2);

	endLoop:
		;
	}
	std::optional<ccl::DeviceInfo> device = {};
	for(auto &devInfo : ccl::Device::available_devices(devices))
	{
		if(devInfo.type == cclDeviceType)
		{
			device = devInfo;
			break;
		}
		if(devInfo.type == ccl::DeviceType::DEVICE_CPU)
			device = devInfo; // Fallback / Default device type
	}
	
	if(device.has_value() == false)
	{
		outErr = "Device '" +ccl::Device::string_from_type(cclDeviceType) +"' is not available!";
		return {}; // No device available
	}

	auto deviceType = createInfo.deviceType;
	if(device->type == ccl::DeviceType::DEVICE_CPU)
		deviceType = unirender::Scene::DeviceType::CPU;
	m_deviceType = deviceType;
	return device;
}

void unirender::cycles::Renderer::InitializeSession(unirender::Scene &scene,const ccl::DeviceInfo &devInfo)
{
	ccl::SceneParams sceneParams {};
	sceneParams.shadingsystem = ccl::SHADINGSYSTEM_SVM;

	auto sessionParams = GetSessionParameters(scene,devInfo);
	m_cclSession = std::make_unique<ccl::Session>(sessionParams,sceneParams);

	auto *cclScene = m_cclSession->scene; // new ccl::Scene{sceneParams,m_cclSession->device}; // Object will be removed automatically by cycles
	cclScene->params.bvh_type = ccl::BVHType::BVH_TYPE_STATIC;
	m_cclScene = cclScene;

	auto &createInfo = scene.GetCreateInfo();
	/*if(createInfo.progressive)
	{
		m_cclSession->update_render_tile_cb = [this](ccl::RenderTile tile,bool param) {
			UpdateRenderTile(GetTileManager(),tile,param);
		};
		m_cclSession->write_render_tile_cb = [this](ccl::RenderTile &tile) {
			WriteRenderTile(GetTileManager(),tile);
		};
	}*/
}

ccl::Object *unirender::cycles::Renderer::FindCclObject(const Object &obj)
{
	auto it = m_objectToCclObject.find(&const_cast<Object&>(obj));
	return (it != m_objectToCclObject.end()) ? it->second : nullptr;
}
ccl::Mesh *unirender::cycles::Renderer::FindCclMesh(const Mesh &mesh)
{
	auto it = m_meshToCcclMesh.find(&const_cast<Mesh&>(mesh));
	return (it != m_meshToCcclMesh.end()) ? it->second : nullptr;
}
ccl::Light *unirender::cycles::Renderer::FindCclLight(const Light &light)
{
	auto it = m_lightToCclLight.find(&const_cast<Light&>(light));
	return (it != m_lightToCclLight.end()) ? it->second : nullptr;
}

void unirender::cycles::Renderer::SyncObject(const unirender::Object &obj)
{
	auto *cclObj = new ccl::Object{};
	m_cclScene->objects.push_back(cclObj);
	m_objectToCclObject[&obj] = cclObj;
	cclObj->set_tfm(ToCyclesTransform(obj.GetPose()));
	auto &mesh = obj.GetMesh();
	cclObj->set_geometry(FindCclMesh(mesh));
	// m_object.tag_update(*scene);

#ifdef ENABLE_MOTION_BLUR_TEST
	m_motionPose.SetOrigin(Vector3{100.f,100.f,100.f});
	m_object.motion.push_back_slow(Scene::ToCyclesTransform(GetMotionPose()));
#endif
}

template<typename TSrc,typename TDst>
	static void copy_vector_to_ccl_array(const std::vector<TSrc> &srcData,ccl::array<TDst> &dstData,const std::function<TDst(const TSrc&)> &translate)
{
	dstData.reserve(dstData.size() +srcData.size());
	for(auto &v : srcData)
		dstData.push_back_reserved(translate(v));
}

template<typename T>
	static void copy_vector_to_ccl_array(const std::vector<T> &srcData,ccl::array<T> &dstData)
{
	copy_vector_to_ccl_array<T,T>(srcData,dstData,[](const T &v) -> T {return v;});
}

template<typename T,typename TCcl>
	static void copy_vector_to_attribute(const std::vector<T> &data,ccl::Attribute &attr,const std::function<TCcl(const T&)> &translate)
{
	if(attr.data_sizeof() != sizeof(TCcl))
		throw std::logic_error{"Data size mismatch"};

	std::vector<TCcl> cclValues;
	cclValues.reserve(data.size());
	for(auto &v : data)
		cclValues.push_back(translate(v));

	attr.resize(cclValues.size());
	auto *ptr = attr.data();
	memcpy(ptr,cclValues.data(),cclValues.size() *sizeof(cclValues[0]));
}

template<typename T,typename TCcl>
	static void initialize_attribute(ccl::Mesh &mesh,ccl::AttributeStandard attrs,const std::vector<T> &data,const std::function<TCcl(const T&)> &translate)
{
	auto *attr = mesh.attributes.add(attrs);
	if(!attr)
		return;
	copy_vector_to_attribute(data,*attr,translate);
}

static ccl::ShaderInput *find_input_socket(ccl::ShaderNode &node,const char *strInput)
{
	for(auto *input : node.inputs)
	{
		if(ccl::string_iequals(input->socket_type.name.string(), strInput))
			return input;
	}
	return nullptr;
}
static const ccl::SocketType *find_type_input(ccl::ShaderNode &node,const char *strInput)
{
	for(auto &input : node.type->inputs)
	{
		if(ccl::string_iequals(input.name.string(), strInput))
			return &input;
	}
	return nullptr;
}
static ccl::ShaderOutput *find_output_socket(ccl::ShaderNode &node,const char *strOutput)
{
	for(auto *output : node.outputs)
	{
		if(ccl::string_iequals(output->socket_type.name.string(), strOutput))
			return output;
	}
	return nullptr;
}
void unirender::cycles::Renderer::SyncMesh(const unirender::Mesh &mesh)
{
	auto *cclMesh = new ccl::Mesh{};
	m_cclScene->geometry.push_back(cclMesh);
	m_meshToCcclMesh[&mesh] = cclMesh;

	cclMesh->name = mesh.GetName();
	cclMesh->reserve_mesh(mesh.GetVertexCount(),mesh.GetTriangleCount());
	for(auto &v : mesh.GetVertices())
		cclMesh->add_vertex(ToCyclesPosition(v));
	auto &tris = mesh.GetTriangles();
	auto &shaderIds = mesh.GetShaders();
	auto &smooth = mesh.GetSmooth();
	auto ntris = tris.size();
	for(auto i=decltype(ntris){0u};i<ntris;i+=3)
		cclMesh->add_triangle(tris[i],tris[i +1],tris[i +2],shaderIds[i /3],smooth[i /3]);

	auto fToFloat4 = [](const ccl::float3 &v) -> ccl::float4 {return ccl::float4{v.x,v.y,v.z,0.f};};
	initialize_attribute<Vector3,ccl::float4>(*cclMesh,ccl::ATTR_STD_VERTEX_NORMAL,mesh.GetVertexNormals(),[&fToFloat4](const Vector3 &v) -> ccl::float4 {return fToFloat4(ToCyclesNormal(v));});
	initialize_attribute<Vector2,ccl::float2>(*cclMesh,ccl::ATTR_STD_UV,mesh.GetUvs(),[](const Vector2 &v) -> ccl::float2 {return ToCyclesUV(v);});
	initialize_attribute<Vector3,ccl::float3>(*cclMesh,ccl::ATTR_STD_UV_TANGENT,mesh.GetUvTangents(),[](const Vector3 &v) -> ccl::float3 {return ToCyclesNormal(v);});
	initialize_attribute<float,float>(*cclMesh,ccl::ATTR_STD_UV_TANGENT_SIGN,mesh.GetUvTangentSigns(),[](const float &v) -> float {return v;});

	auto *attrT = cclMesh->attributes.add(ccl::ATTR_STD_UV_TANGENT);
	if(attrT)
		attrT->name = "orco" +Mesh::TANGENT_POSTFIX;

	auto *attrTS = cclMesh->attributes.add(ccl::ATTR_STD_UV_TANGENT_SIGN);
	if(attrTS)
		attrTS->name = "orco" +Mesh::TANGENT_SIGN_POSTIFX;

	if(mesh.HasAlphas())
	{
		auto &alphas = mesh.GetAlphas();
		cclMesh->attributes.add(ALPHA_ATTRIBUTE_TYPE);
		initialize_attribute<float,float>(*cclMesh,ALPHA_ATTRIBUTE_TYPE,*alphas,[](const float &v) -> float {return v;});
	}

	auto &shaders = mesh.GetSubMeshShaders();
	auto usedShaders = cclMesh->get_used_shaders();
	usedShaders.resize(shaders.size());

	auto apiData = GetApiData();
	auto useDebugMeshShader = false;
	apiData.GetFromPath("cycles/debug/useDebugMeshShader")(useDebugMeshShader);
	if(useDebugMeshShader)
	{
		for(auto i=decltype(shaders.size()){0u};i<shaders.size();++i)
		{
			auto *shader = AddDebugShader();
			usedShaders[i] = shader;
		}
	}
	else
	{
		for(auto i=decltype(shaders.size()){0u};i<shaders.size();++i)
		{
			auto desc = shaders.at(i)->GetActivePassNode();
			if(desc == nullptr)
				desc = GroupNodeDesc::Create(m_scene->GetShaderNodeManager()); // Just create a dummy node
			auto cclShader = CCLShader::Create(*this,*desc);
			if(cclShader == nullptr)
				throw std::logic_error{"Mesh shader must never be NULL!"};
			if(cclShader)
				usedShaders[i] = **cclShader;
		}
	}
	cclMesh->set_used_shaders(usedShaders);

	// TODO: We should be using the tangent values from m_tangents / m_tangentSigns
	// but their coordinate system needs to be converted for Cycles.
	// For now we'll just re-compute the tangents here.
	compute_tangents(cclMesh,true,true);

	if(cclMesh->need_attribute(m_cclScene,ccl::ATTR_STD_GENERATED))
	{
		auto *attr = cclMesh->attributes.add(ccl::ATTR_STD_GENERATED);
		memcpy(attr->data_float3(), cclMesh->get_verts().data(), sizeof(ccl::float3) *cclMesh->get_verts().size());
	}
}

void unirender::cycles::Renderer::SyncCamera(const unirender::Camera &cam,bool update)
{
	auto &cclCam = *(*this)->camera;

	switch(cam.GetType())
	{
	case unirender::Camera::CameraType::Perspective:
		cclCam.set_camera_type(ccl::CameraType::CAMERA_PERSPECTIVE);
		break;
	case unirender::Camera::CameraType::Orthographic:
		cclCam.set_camera_type(ccl::CameraType::CAMERA_ORTHOGRAPHIC);
		break;
	case unirender::Camera::CameraType::Panorama:
		cclCam.set_camera_type(ccl::CameraType::CAMERA_PANORAMA);
		break;
	}

	switch(cam.GetPanoramaType())
	{
	case unirender::Camera::PanoramaType::Equirectangular:
		cclCam.set_panorama_type(ccl::PanoramaType::PANORAMA_EQUIRECTANGULAR);
		break;
	case unirender::Camera::PanoramaType::FisheyeEquidistant:
		cclCam.set_panorama_type(ccl::PanoramaType::PANORAMA_FISHEYE_EQUIDISTANT);
		break;
	case unirender::Camera::PanoramaType::FisheyeEquisolid:
		cclCam.set_panorama_type(ccl::PanoramaType::PANORAMA_FISHEYE_EQUISOLID);
		break;
	case unirender::Camera::PanoramaType::Mirrorball:
		cclCam.set_panorama_type(ccl::PanoramaType::PANORAMA_MIRRORBALL);
		break;
	}

	cclCam.set_full_width(cam.GetWidth());
	cclCam.set_full_height(cam.GetHeight());
	cclCam.set_nearclip(cam.GetNearZ());
	cclCam.set_farclip(cam.GetFarZ());
	cclCam.set_fov(umath::deg_to_rad(cam.GetFov()));
	cclCam.set_focaldistance(cam.GetFocalDistance());
	cclCam.set_aperturesize(cam.GetApertureSize());
	cclCam.set_aperture_ratio(cam.GetApertureRatio());
	cclCam.set_blades(cam.GetBladeCount());
	cclCam.set_bladesrotation(umath::deg_to_rad(cam.GetBladesRotation()));
	cclCam.set_interocular_distance(units::convert<units::length::millimeter,units::length::meter>(cam.GetInterocularDistance()));
	cclCam.set_longitude_max(umath::deg_to_rad(cam.GetLongitudeMax()));
	cclCam.set_longitude_min(umath::deg_to_rad(cam.GetLongitudeMin()));
	cclCam.set_latitude_max(umath::deg_to_rad(cam.GetLatitudeMax()));
	cclCam.set_latitude_min(umath::deg_to_rad(cam.GetLatitudeMin()));
	cclCam.set_use_spherical_stereo(cam.IsStereoscopic());

#ifdef ENABLE_MOTION_BLUR_TEST
	SetShutterTime(1.f);
#endif

	if(cam.IsDofEnabled() == false)
		cclCam.set_aperturesize(0.f);
	auto pose = cam.GetPose();
	if(cam.GetType() == unirender::Camera::CameraType::Panorama)
	{
		auto rot = cam.GetRotation();
		switch(cam.GetPanoramaType())
		{
		case unirender::Camera::PanoramaType::Mirrorball:
			rot *= uquat::create(EulerAngles{-90.f,0.f,0.f});
			break;
		case unirender::Camera::PanoramaType::FisheyeEquisolid:
			cclCam.set_fisheye_lens(10.5f);
			cclCam.set_fisheye_fov(180.f);
			// No break is intentional!
		default:
			rot *= uquat::create(EulerAngles{-90.f,-90.f,0.f});
			break;
		}
		pose.SetRotation(rot);
	}

	cclCam.set_matrix(ToCyclesTransform(pose,true));
	cclCam.compute_auto_viewplane();
	
	if(update)
		return;
	//
	std::cout<<"Camera settings:"<<std::endl;
	std::cout<<"Width: "<<cclCam.get_full_width()<<std::endl;
	std::cout<<"Height: "<<cclCam.get_full_height()<<std::endl;
	std::cout<<"NearZ: "<<cclCam.get_nearclip()<<std::endl;
	std::cout<<"FarZ: "<<cclCam.get_farclip()<<std::endl;
	std::cout<<"FOV: "<<umath::rad_to_deg(cclCam.get_fov())<<std::endl;
	std::cout<<"Focal Distance: "<<cclCam.get_focaldistance()<<std::endl;
	std::cout<<"Aperture Size: "<<cclCam.get_aperturesize()<<std::endl;
	std::cout<<"Aperture Ratio: "<<cclCam.get_aperture_ratio()<<std::endl;
	std::cout<<"Blades: "<<cclCam.get_blades()<<std::endl;
	std::cout<<"Blades Rotation: "<<cclCam.get_bladesrotation()<<std::endl;
	std::cout<<"Interocular Distance: "<<cclCam.get_interocular_distance()<<std::endl;
	std::cout<<"Longitude Max: "<<cclCam.get_longitude_max()<<std::endl;
	std::cout<<"Longitude Min: "<<cclCam.get_longitude_min()<<std::endl;
	std::cout<<"Latitude Max: "<<cclCam.get_latitude_max()<<std::endl;
	std::cout<<"Latitude Min: "<<cclCam.get_latitude_min()<<std::endl;
	std::cout<<"Use Spherical Stereo: "<<cclCam.get_use_spherical_stereo()<<std::endl;
	std::cout<<"Matrix: ";
	auto first = true;
	for(uint8_t i=0;i<3;++i)
	{
		for(uint8_t j=0;j<4;++j)
		{
			auto v = cclCam.get_matrix()[i][j];
			if(first)
				first = false;
			else
				std::cout<<",";
			std::cout<<v;
		}
	}
	std::cout<<std::endl;
	//

	cclCam.need_flags_update = true;
	cclCam.update(&**this);
}

void unirender::cycles::Renderer::SyncLight(unirender::Scene &scene,const unirender::Light &light,bool update)
{
	ccl::Light *cclLight = nullptr;
	if(update)
	{
		auto it = m_lightToCclLight.find(&light);
		if(it == m_lightToCclLight.end())
			return;
		cclLight = it->second;
	}
	else
	{
		cclLight = new ccl::Light{}; // Object will be removed automatically by cycles
		m_cclScene->lights.push_back(cclLight);
		m_lightToCclLight[&light] = cclLight;
	}
	cclLight->set_tfm(ccl::transform_identity());
	switch(light.GetType())
	{
	case unirender::Light::Type::Spot:
		cclLight->set_light_type(ccl::LightType::LIGHT_SPOT);
		break;
	case unirender::Light::Type::Directional:
		cclLight->set_light_type(ccl::LightType::LIGHT_DISTANT);
		break;
	case unirender::Light::Type::Area:
		cclLight->set_light_type(ccl::LightType::LIGHT_AREA);
		break;
	case unirender::Light::Type::Background:
		cclLight->set_light_type(ccl::LightType::LIGHT_BACKGROUND);
		break;
	case unirender::Light::Type::Triangle:
		cclLight->set_light_type(ccl::LightType::LIGHT_TRIANGLE);
		break;
	case unirender::Light::Type::Point:
	default:
		cclLight->set_light_type(ccl::LightType::LIGHT_POINT);
		break;
	}

	switch(light.GetType())
	{
	case unirender::Light::Type::Point:
	{
		break;
	}
	case unirender::Light::Type::Spot:
	{
		auto &rot = light.GetRotation();
		auto forward = uquat::forward(rot);
		cclLight->set_dir(ToCyclesNormal(forward));
		auto innerConeAngle = umath::deg_to_rad(light.GetInnerConeAngle());
		auto outerConeAngle = umath::deg_to_rad(light.GetOuterConeAngle());
		cclLight->set_spot_smooth((outerConeAngle > 0.f) ? (1.f -innerConeAngle /outerConeAngle) : 1.f);
		cclLight->set_spot_angle(outerConeAngle);
		break;
	}
	case unirender::Light::Type::Directional:
	{
		auto &rot = light.GetRotation();
		auto forward = uquat::forward(rot);
		cclLight->set_dir(ToCyclesNormal(forward));
		break;
	}
	case unirender::Light::Type::Area:
	{
		auto &axisU = light.GetAxisU();
		auto &axisV = light.GetAxisV();
		auto sizeU = light.GetSizeU();
		auto sizeV = light.GetSizeV();
		cclLight->set_axisu(ToCyclesNormal(axisU));
		cclLight->set_axisv(ToCyclesNormal(axisV));
		cclLight->set_sizeu(ToCyclesLength(sizeU));
		cclLight->set_sizev(ToCyclesLength(sizeV));
		cclLight->set_round(light.IsRound());

		auto &rot = light.GetRotation();
		auto forward = uquat::forward(rot);
		cclLight->set_dir(ToCyclesNormal(forward));
		break;
	}
	case unirender::Light::Type::Background:
	{
		break;
	}
	case unirender::Light::Type::Triangle:
	{
		break;
	}
	}

	auto lightType = (light.GetType() == unirender::Light::Type::Spot) ? util::pragma::LightType::Spot : (light.GetType() == unirender::Light::Type::Directional) ? util::pragma::LightType::Directional : util::pragma::LightType::Point;
	auto watt = (lightType == util::pragma::LightType::Spot) ? ulighting::cycles::lumen_to_watt_spot(light.GetIntensity(),light.GetColor(),light.GetOuterConeAngle()) :
		(lightType == util::pragma::LightType::Point) ? ulighting::cycles::lumen_to_watt_point(light.GetIntensity(),light.GetColor()) :
		ulighting::cycles::lumen_to_watt_area(light.GetIntensity(),light.GetColor());

	// Multiple importance sampling. It's disabled by default for some reason, but it's usually best to keep it on.
	// cclLight->set_use_mis(true);
	// Update: Enabling MIS turns all lights into bright white circles in Cycles X, so it's disabled for now.

	//static float lightIntensityFactor = 10.f;
	//watt *= lightIntensityFactor;

	static auto lightIntensityMultiplier = 200.f;
	watt *= scene.GetLightIntensityFactor() *lightIntensityMultiplier;
	auto &color = light.GetColor();
	cclLight->set_strength(ccl::float3{color.r,color.g,color.b} *watt);
	cclLight->set_size(ToCyclesLength(light.GetSize()));
	cclLight->set_co(ToCyclesPosition(light.GetPos()));

	cclLight->set_max_bounces(1'024);
	cclLight->set_map_resolution(2'048);
	cclLight->tag_update(m_cclScene);
	// 
	// Test
	/*m_light->strength = ccl::float3{0.984539f,1.f,0.75f} *40.f;
	m_light->size = 0.25f;
	m_light->max_bounces = 1'024;
	m_light->type = ccl::LightType::LIGHT_POINT;*/

	if(update)
		return;
	auto desc = GroupNodeDesc::Create(scene.GetShaderNodeManager());
	auto &outputNode = desc->AddNode(NODE_OUTPUT);
	auto &nodeEmission = desc->AddNode(NODE_EMISSION);
	//nodeEmission->SetInputArgument<float>("strength",watt);
	//nodeEmission->SetInputArgument<ccl::float3>("color",ccl::float3{1.f,1.f,1.f});
	desc->Link(nodeEmission.GetOutputSocket("emission"),outputNode.GetInputSocket("surface"));
	cclLight->set_shader(**CCLShader::Create(*this,*desc));
}

ccl::BufferParams unirender::cycles::Renderer::GetBufferParameters() const
{
	auto &cam = m_scene->GetCamera();
	ccl::BufferParams bufferParams {};
	bufferParams.width = cam.GetWidth();
	bufferParams.height = cam.GetHeight();
	bufferParams.full_width = cam.GetWidth();
	bufferParams.full_height = cam.GetHeight();
	SetupRenderSettings(*m_cclScene,*m_cclSession,bufferParams,m_renderMode,m_scene->GetSceneInfo().maxTransparencyBounces);
	return bufferParams;
}

float unirender::cycles::Renderer::GetProgress() const
{
	return m_cclSession->progress.get_progress();
}
bool unirender::cycles::Renderer::Stop() {return false;}
bool unirender::cycles::Renderer::Pause() {return false;}
bool unirender::cycles::Renderer::Resume() {return false;}
bool unirender::cycles::Renderer::Suspend() {return false;}
bool unirender::cycles::Renderer::Export(const std::string &path) {return false;}
void unirender::cycles::Renderer::Wait()
{
	if(m_cclSession)
		m_cclSession->wait();
}

void unirender::cycles::Renderer::ApplyPostProcessing(uimg::ImageBuffer &imgBuffer,unirender::Scene::RenderMode renderMode)
{
	// For some reason the image is flipped horizontally when rendering an image,
	// so we'll just flip it the right way here
	auto flipHorizontally = unirender::Scene::IsRenderSceneMode(renderMode);
	if(m_cclScene->camera->get_camera_type() == ccl::CameraType::CAMERA_PANORAMA)
	{
		switch(m_cclScene->camera->get_panorama_type())
		{
		case ccl::PanoramaType::PANORAMA_EQUIRECTANGULAR:
		case ccl::PanoramaType::PANORAMA_FISHEYE_EQUIDISTANT:
			flipHorizontally = false; // I have no idea why some types have to be flipped and others don't
			break;
		}
	}
	if(flipHorizontally)
		imgBuffer.FlipHorizontally();

	// We will also always have to flip the image vertically, since the data seems to be bottom->top and we need it top->bottom
	if(renderMode != Scene::RenderMode::BakeDiffuseLighting)
		imgBuffer.FlipVertically();
	imgBuffer.ClearAlpha();
}

std::optional<uint32_t> unirender::cycles::Renderer::FindCCLObjectId(const ccl::Object &o) const
{
	auto it = std::find(m_cclScene->objects.begin(),m_cclScene->objects.end(),&o);
	return (it != m_cclScene->objects.end()) ? (it -m_cclScene->objects.begin()) : std::optional<uint32_t>{};
}

struct Options {
  ccl::Session *session;
  ccl::Scene *scene;
  ccl::string filepath;
  int width, height;
  ccl::SceneParams scene_params;
  ccl::SessionParams session_params;
  bool quiet;
  bool show_help, interactive, pause;
  ccl::string output_filepath;
  ccl::string output_pass;
};


#include "app/cycles_xml.h"
static void scene_init(Options &options)
{
  options.scene = options.session->scene;

  /* Read XML */
 ccl::xml_read_file(options.scene, options.filepath.c_str());

  /* Camera width/height override? */
  if (!(options.width == 0 || options.height == 0)) {
    options.scene->camera->set_full_width(options.width);
    options.scene->camera->set_full_height(options.height);
  }
  else {
    options.width = options.scene->camera->get_full_width();
    options.height = options.scene->camera->get_full_height();
  }

  /* Calculate Viewplane */
  options.scene->camera->compute_auto_viewplane();
}

static ccl::BufferParams &session_buffer_params(Options &opts)
{
  static ccl::BufferParams buffer_params;
  buffer_params.width = opts.width;
  buffer_params.height = opts.height;
  buffer_params.full_width = opts.width;
  buffer_params.full_height = opts.height;

  return buffer_params;
}

static void session_print(const ccl::string &str)
{
  /* print with carriage return to overwrite previous */
 // printf("\r%s", str.c_str());

  /* add spaces to overwrite longer previous print */
  static int maxlen = 0;
  int len = str.size();
  maxlen = ccl::max(len, maxlen);

  //for (int i = len; i < maxlen; i++)
  //  printf(" ");

  /* flush because we don't write an end of line */
  //fflush(stdout);
}

static void session_print_status(Options &opts)
{
  ccl::string status, substatus;

  /* get status */
  double progress = opts.session->progress.get_progress();
  opts.session->progress.get_status(status, substatus);

  if (substatus != "")
    status += ": " + substatus;

  /* print status */
  //status = ccl::string_printf("Progress %05.2f   %s", (double)progress * 100, status.c_str());
  //session_print(status);
}

#include <app/oiio_output_driver.h>

class COIIOOutputDriver : public ccl::OutputDriver {
 public:
  typedef ccl::function<void(const ccl::string &)> LogFunction;

  COIIOOutputDriver(const ccl::string_view filepath, const ccl::string_view pass, LogFunction log);
  virtual ~COIIOOutputDriver();

  void write_render_tile(const Tile &tile) override;

 protected:
  ccl::string filepath_;
  ccl::string pass_;
  LogFunction log_;
};

COIIOOutputDriver::COIIOOutputDriver(const ccl::string_view filepath,
                                   const ccl::string_view pass,
                                   LogFunction log)
    : filepath_(filepath), pass_(pass), log_(log)
{
}

COIIOOutputDriver::~COIIOOutputDriver()
{
}

void COIIOOutputDriver::write_render_tile(const Tile &tile)
{
  /* Only write the full buffer, no intermediate tiles. */
  if (!(tile.size == tile.full_size)) {
    return;
  }

  //log_(ccl::string_printf("Writing image %s", filepath_.c_str()));

  ccl::unique_ptr<ccl::ImageOutput> image_output(ccl::ImageOutput::create(filepath_));
  if (image_output == nullptr) {
    //log_("Failed to create image file");
    return;
  }

  const int width = tile.size.x;
  const int height = tile.size.y;

  ccl::ImageSpec spec(width, height, 4, ccl::TypeDesc::FLOAT);
  if (!image_output->open(filepath_, spec)) {
    //log_("Failed to create image file");
    return;
  }

  ccl::vector<float> pixels(width * height * 4);
  if (!tile.get_pass_pixels(pass_, 4, pixels.data())) {
    //log_("Failed to read render pass pixels");
    return;
  }

  /* Manipulate offset and stride to convert from bottom-up to top-down convention. */
  image_output->write_image(ccl::TypeDesc::FLOAT,
                            pixels.data() + (height - 1) * width * 4,
                            ccl::AutoStride,
                            -width * 4 * sizeof(float),
                            ccl::AutoStride);
  image_output->close();
}

void unirender::cycles::Renderer::AddDebugSky()
{
	auto *shader = m_cclScene->default_background;
	auto *graph = new ccl::ShaderGraph();

	const ccl::NodeType *skyTexNodeType = ccl::NodeType::find(ccl::ustring{"sky_texture"});
	auto skyTex = (ccl::SkyTextureNode*)skyTexNodeType->create(skyTexNodeType);
	skyTex->set_owner(graph);
	skyTex->set_sky_type(ccl::NodeSkyType::NODE_SKY_HOSEK);
	skyTex->name = ccl::ustring{"tex"};
	graph->add(skyTex);

	const ccl::NodeType *bgShaderNodeType = ccl::NodeType::find(ccl::ustring{"background_shader"});
	auto bgShader = (ccl::BackgroundNode *)bgShaderNodeType->create(bgShaderNodeType);
	bgShader->set_owner(graph);
	bgShader->set_strength(8.f);
	bgShader->set_color({1.f,0.f,0.f});
	bgShader->name = ccl::ustring{"bg"};
	graph->add(bgShader);

	graph->connect(find_output_socket(*skyTex,"color"),find_input_socket(*bgShader,"color"));
	graph->connect(find_output_socket(*bgShader,"background"),find_input_socket(*graph->output(),"surface"));

	shader->set_graph(graph);
	shader->tag_update(m_cclScene);
}

ccl::Mesh *unirender::cycles::Renderer::AddDebugMesh()
{
	auto *cclMesh = new ccl::Mesh{};
	m_cclScene->geometry.push_back(cclMesh);

	cclMesh->name = "floor";
	auto *mesh = cclMesh;
	ccl::array<ccl::float3> P_array {};
	P_array.push_back_slow(ccl::float3{-3.f,3.f,0.f});
	P_array.push_back_slow(ccl::float3{3.f,3.f,0.f});
	P_array.push_back_slow(ccl::float3{3.f,-3.f,0.f});
	P_array.push_back_slow(ccl::float3{-3.f,-3.f,0.f});
	mesh->set_verts(P_array);

	size_t num_triangles = 0;
	ccl::vector<int> nverts {};
	nverts.push_back(4);
	for (size_t i = 0; i < nverts.size(); i++)
	num_triangles += nverts[i] - 2;
	mesh->reserve_mesh(mesh->get_verts().size(), num_triangles);

	/* create triangles */
	int index_offset = 0;

	ccl::vector<int> verts;
	verts.push_back(0);
	verts.push_back(1);
	verts.push_back(2);
	verts.push_back(3);
	int ishader = 0;
	bool smooth = true;
	for (size_t i = 0; i < nverts.size(); i++) {
		for (int j = 0; j < nverts[i] - 2; j++) {
			int v0 = verts[index_offset];
			int v1 = verts[index_offset + j + 1];
			int v2 = verts[index_offset + j + 2];

			assert(v0 < (int)P.size());
			assert(v1 < (int)P.size());
			assert(v2 < (int)P.size());

			mesh->add_triangle(v0, v1, v2, ishader, smooth);
		}

		index_offset += nverts[i];
	}

	if (mesh->need_attribute(m_cclScene, ccl::ATTR_STD_GENERATED)) {
		ccl::Attribute *attr = mesh->attributes.add(ccl::ATTR_STD_GENERATED);
		memcpy(
			attr->data_float3(), mesh->get_verts().data(), sizeof(ccl::float3) * mesh->get_verts().size()
		);
	}
	return cclMesh;
}
ccl::Object *unirender::cycles::Renderer::AddDebugObject()
{
	auto *mesh = AddDebugMesh();
	ccl::Object *object = new ccl::Object();
	object->set_geometry(mesh);
	ccl::Transform t = ccl::transform_identity();
	object->set_tfm(t);
	m_cclScene->objects.push_back(object);
	return object;
}
void unirender::cycles::Renderer::AddDebugLight()
{
	auto *shader = new ccl::Shader{};
	shader->name = "point_shader";
	ccl::ShaderGraph *graph = new ccl::ShaderGraph();
	
	const ccl::NodeType *emissionType = ccl::NodeType::find(ccl::ustring{"emission"});
	auto emissionNode = (ccl::EmissionNode *)emissionType->create(emissionType);
	emissionNode->set_owner(graph);
	emissionNode->name = ccl::ustring{"emission"};
	emissionNode->set_color(ccl::float3{0.8f,0.1f,0.1f} *100.f);
	graph->add(emissionNode);

	graph->connect( find_output_socket(*emissionNode,"emission"),find_input_socket(*graph->output(),"surface"));
	
	shader->set_graph(graph);
	shader->tag_update(m_cclScene);
	m_cclScene->shaders.push_back(shader);

	//

	auto *light = new ccl::Light{};
	m_cclScene->lights.push_back(light);
	light->set_light_type(ccl::LightType::LIGHT_POINT);
	light->set_shader(shader);
	light->set_size(1.f);
	light->set_co({0.f,0.f,1.f});
}
ccl::Shader *unirender::cycles::Renderer::AddDebugShader()
{
	auto *shader = new ccl::Shader{};
	shader->name = "shader_test";
	ccl::ShaderGraph *graph = new ccl::ShaderGraph();
	
	const ccl::NodeType *nodeTypeGlossy = ccl::NodeType::find(ccl::ustring{"glossy_bsdf"});
	auto glossyNode = (ccl::GlossyBsdfNode *)nodeTypeGlossy->create(nodeTypeGlossy);
	glossyNode->set_owner(graph);
	glossyNode->name = ccl::ustring{"floor_closure2"};
	glossyNode->set(*find_type_input(*glossyNode,"roughness"),0.2f);
	glossyNode->set(*find_type_input(*glossyNode,"distribution"),"beckmann");
	graph->add(glossyNode);

	const ccl::NodeType *nodeTypeCheckerTex = ccl::NodeType::find(ccl::ustring{"checker_texture"});
	auto checkerNode = (ccl::CheckerTextureNode *)nodeTypeCheckerTex->create(nodeTypeCheckerTex);
	checkerNode->set_owner(graph);
	checkerNode->name = ccl::ustring{"checker2"};
	checkerNode->set(*find_type_input(*checkerNode,"color1"),ccl::float3{0.8f,0.8f,0.8f});
	checkerNode->set(*find_type_input(*checkerNode,"color2"),ccl::float3{1.f,0.1f,0.1f});
	graph->add(checkerNode);
		
	graph->connect( find_output_socket(*checkerNode,"color"),find_input_socket(*glossyNode,"color"));
	graph->connect( find_output_socket(*glossyNode,"bsdf"),find_input_socket(*graph->output(),"surface"));
	
	shader->set_graph(graph);
	shader->tag_update(m_cclScene);
	m_cclScene->shaders.push_back(shader);
	return shader;
}

void unirender::cycles::Renderer::InitializeDebugScene(const std::string &fileName,const std::vector<std::string> &xmlFileNames)
{
	Options opts {};

	opts.width = 1024;
	opts.height = 512;
	opts.output_filepath = fileName;
	opts.session = NULL;
	opts.quiet = false;
	opts.session_params.use_auto_tile = false;
	opts.session_params.tile_size = 16;
	opts.session_params.samples = 20;

	opts.output_pass = "combined";
	opts.session = m_cclSession.get();
	opts.scene = opts.session->scene;

	auto &cam = GetScene().GetCamera();
	SyncCamera(cam);

	PopulateDebugScene();
	for(auto &filepath : xmlFileNames)
		ccl::xml_read_file(opts.scene, filepath.c_str());
	
	if (!opts.output_filepath.empty()) {
		opts.session->set_output_driver(make_unique<COIIOOutputDriver>(
		opts.output_filepath, opts.output_pass, session_print));
	}

	opts.session->progress.set_update_callback([this]() {std::cout<<"Progress: "<<m_cclSession->progress.get_progress()<<","<<m_cclSession->progress.get_cancel_message()<<","<<m_cclSession->progress.get_error_message()<<std::endl;});
	  
	/* add pass for output. */
	ccl::Pass *pass = opts.scene->create_node<ccl::Pass>();
	pass->set_name(ccl::ustring(opts.output_pass.c_str()));
	pass->set_type(ccl::PASS_COMBINED);

	auto useOptix = false;
	if(useOptix)
	{
		using namespace ccl;
		auto devices = ccl::Device::available_devices(DEVICE_MASK(ccl::DeviceType::DEVICE_OPTIX));
		opts.session_params.device = devices.front();
	}
	opts.session_params.use_auto_tile = false;
	opts.session_params.tile_size = 0;
	opts.session_params.background = true;

	opts.session->reset(opts.session_params, session_buffer_params(opts));
	opts.session->start();

	opts.session->wait();

	opts.session = nullptr;
	m_cclSession = nullptr;
}

void unirender::cycles::Renderer::PopulateDebugScene()
{
	AddDebugSky();
	auto *obj = AddDebugObject();
	auto *shader = AddDebugShader();
	auto *mesh = obj->get_geometry();
	AddDebugLight();

	ccl::array<ccl::Node *> used_shaders = mesh->get_used_shaders();
	used_shaders.push_back_slow(shader);
	mesh->set_used_shaders(used_shaders);
}

bool unirender::cycles::Renderer::BeginSceneEdit() {return true;}
bool unirender::cycles::Renderer::EndSceneEdit() {return true;}
bool unirender::cycles::Renderer::SyncEditedActor(const util::Uuid &uuid)
{
	auto *actor = FindActor(uuid);
	if(!actor)
		return false;
	if(typeid(*actor) == typeid(Camera))
		SyncCamera(static_cast<Camera&>(*actor),true);
	else if(typeid(*actor) == typeid(Light))
		SyncLight(*m_scene,static_cast<Light&>(*actor),true);
	else
		return false;
	umath::set_flag(m_stateFlags,StateFlags::ReloadSessionScheduled);
	return true;
}

bool unirender::cycles::Renderer::Initialize(unirender::Scene &scene,std::string &outErr)
{
	auto devInfo = InitializeDevice(scene,outErr);
	if(devInfo.has_value() == false)
		return false;

	auto apiData = GetApiData();
	auto udmDebugStandalone = apiData.GetFromPath("cycles/debug/debugStandalone");
	if(udmDebugStandalone)
	{
		std::string xmlFile;
		udmDebugStandalone["xmlFile"](xmlFile);

		std::string outputFile;
		udmDebugStandalone["outputFile"](outputFile);

		uint32_t samples = 20;
		udmDebugStandalone["samples"](samples);

		std::string deviceName = "CPU";
		udmDebugStandalone["device"](deviceName);

		std::cout<<"Selected device: "<<deviceName<<std::endl;
		auto strSamples = std::to_string(samples);
		const char *args[] = {
			"",
			xmlFile.c_str(),
			"--output",
			outputFile.c_str(),
			"--samples",
			strSamples.c_str(),
			"--device",
			deviceName.c_str(),
			"--background"
		}; 
		cycles_standalone_test(9,args,false);
		return false;
	}

	auto nativeDenoising = false;
	auto udmDebug = apiData.GetFromPath("cycles/debug");
	udmDebug["nativeDenoising"](nativeDenoising);
	umath::set_flag(m_stateFlags,StateFlags::NativeDenoising,nativeDenoising);
	
	auto denoiserType = ccl::DenoiserType::DENOISER_NONE;
	if(!umath::is_flag_set(m_stateFlags,StateFlags::NativeDenoising))
	{
		auto denoiseMode = m_scene->GetDenoiseMode();
		std::vector<ccl::DenoiserType> denoisePreferenceOrder;
		if(denoiseMode == Scene::DenoiseMode::AutoFast || denoiseMode == Scene::DenoiseMode::AutoDetailed || denoiseMode == Scene::DenoiseMode::Optix)
		{
			denoisePreferenceOrder.push_back(ccl::DenoiserType::DENOISER_OPTIX);
			denoisePreferenceOrder.push_back(ccl::DenoiserType::DENOISER_OPENIMAGEDENOISE);
		}
		else if(denoiseMode == Scene::DenoiseMode::OpenImage)
		{
			denoisePreferenceOrder.push_back(ccl::DenoiserType::DENOISER_OPENIMAGEDENOISE);
			denoisePreferenceOrder.push_back(ccl::DenoiserType::DENOISER_OPTIX);
		}
	
		auto availableDenoisers = devInfo->denoisers;
		for(auto type : denoisePreferenceOrder)
		{
			if((availableDenoisers &type) != 0)
			{
				denoiserType = type;
				break;
			}
		}

		if(denoiserType == ccl::DenoiserType::DENOISER_NONE && denoiseMode != Scene::DenoiseMode::None)
			umath::set_flag(m_stateFlags,StateFlags::NativeDenoising); // No Cycles denoising available; Fall back to native denoising
	}

	switch(scene.GetRenderMode())
	{
	case unirender::Scene::RenderMode::SceneAlbedo:
		AddOutput(OUTPUT_ALBEDO);
		break;
	case unirender::Scene::RenderMode::SceneNormals:
		AddOutput(OUTPUT_NORMAL);
		break;
	case unirender::Scene::RenderMode::SceneDepth:
		AddOutput(OUTPUT_DEPTH);
		break;
	case unirender::Scene::RenderMode::BakeAmbientOcclusion:
	case unirender::Scene::RenderMode::BakeNormals:
	case unirender::Scene::RenderMode::BakeDiffuseLighting:
		AddOutput(OUTPUT_COLOR);
		break;
	case unirender::Scene::RenderMode::RenderImage:
	{
		AddOutput(OUTPUT_COLOR);
		if(umath::is_flag_set(m_stateFlags,StateFlags::NativeDenoising))
		{
			AddOutput(OUTPUT_ALBEDO);
			AddOutput(OUTPUT_NORMAL);
		}
		break;
	}
	default:
		return false;
	}

	InitializeSession(scene,*devInfo);
	auto &createInfo = scene.GetCreateInfo();
	auto bufferParams = GetBufferParameters();

	if(denoiserType == ccl::DenoiserType::DENOISER_NONE)
		m_cclScene->integrator->set_use_denoise(false);
	else
	{
		m_cclScene->integrator->set_use_denoise(true);
		m_cclScene->integrator->set_denoiser_type(denoiserType);
		m_cclScene->integrator->set_denoise_start_sample(1);
	}

	apiData = GetApiData();
	auto renderDebugScene = false;
	auto udmDebugScene = apiData.GetFromPath("cycles/debug/debugScene");
	udmDebugScene["enabled"](renderDebugScene);
	if(renderDebugScene)
	{
		std::string outputFileName;
		if(!udmDebugScene["outputFileName"](outputFileName))
		{
			outputFileName = "temp/cycles/";
			filemanager::create_path(outputFileName);
			outputFileName = util::get_program_path() +'/' +outputFileName +"debug_scene.png";
		}
		std::vector<std::string> xmlFiles;
		udmDebugScene["xmlFiles"](xmlFiles);
		InitializeDebugScene(outputFileName,xmlFiles);
		return false;
	}

	m_sessionParams = m_cclSession->params;
	m_bufferParams = bufferParams;
	m_cclSession->reset(m_sessionParams,m_bufferParams);

	if(m_scene->GetSceneInfo().sky.empty() == false)
		AddSkybox(m_scene->GetSceneInfo().sky);
	umath::set_flag(m_stateFlags,StateFlags::RenderingStarted);
	
	auto &mdlCache = m_renderData.modelCache;
	mdlCache->GenerateData();
	uint32_t numObjects = 0;
	uint32_t numMeshes = 0;
	for(auto &chunk : mdlCache->GetChunks())
	{
		numObjects += chunk.GetObjects().size();
		numMeshes += chunk.GetMeshes().size();
	}
	m_cclScene->objects.reserve(m_cclScene->objects.size() +numObjects);
	m_cclScene->geometry.reserve(m_cclScene->geometry.size() +numMeshes);

	auto &cam = scene.GetCamera();
	SyncCamera(cam);

	auto addDebugLight = false;
	auto udmDebugLight = apiData.GetFromPath("cycles/debug/debug_light")(addDebugLight);
	if(addDebugLight)
		AddDebugLight();
	else
	{
		// Note: Lights and objects have to be initialized before shaders, because they may
		// create additional shaders.
		auto &lights = scene.GetLights();
		m_cclScene->lights.reserve(lights.size());
		for(auto &light : lights)
		{
			light->Finalize(scene);
			SyncLight(scene,*light);
		}
	}
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
			o->Finalize(*m_scene);
		for(auto &o : chunk.GetMeshes())
			o->Finalize(*m_scene);
	}
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetMeshes())
			SyncMesh(*o);
	}
	for(auto &chunk : mdlCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
			SyncObject(*o);
	}
	for(auto &shader : m_renderData.shaderCache->GetShaders())
		shader->Finalize();
	for(auto &cclShader : m_cclShaders)
		cclShader->Finalize(*m_scene);
	UpdateActorMap();

	constexpr auto validate = false;
	if constexpr(validate)
	{
		for(auto &chunk : m_renderData.modelCache->GetChunks())
		{
			for(auto &o : chunk.GetObjects())
			{
				auto &mesh = o->GetMesh();
				mesh.Validate();
			}
		}
	}
	
	// TODO: Move this to shared code
	if(createInfo.colorTransform.has_value())
	{
		std::string err;
		ColorTransformProcessorCreateInfo ctpCreateInfo {};
		ctpCreateInfo.config = createInfo.colorTransform->config;
		ctpCreateInfo.lookName = createInfo.colorTransform->lookName;
		ctpCreateInfo.bitDepth = ColorTransformProcessorCreateInfo::BitDepth::Float16;
		m_colorTransformProcessor = create_color_transform_processor(ctpCreateInfo,err,0.f,m_scene->GetGamma());
		if(m_colorTransformProcessor == nullptr)
			m_scene->HandleError("Unable to initialize color transform processor: " +err);
	}

	auto &sceneInfo = m_scene->GetSceneInfo();
	if(createInfo.progressive && GetTileSize() > 0)
	{
		auto w = m_cclScene->camera->get_full_width();
		auto h = m_cclScene->camera->get_full_height();
		m_tileManager.Initialize(w,h,GetTileSize(),GetTileSize(),m_deviceType == Scene::DeviceType::CPU,createInfo.exposure,m_scene->GetGamma(),m_colorTransformProcessor.get());
		bool flipHorizontally = true;
		if(m_cclScene->camera->get_camera_type() == ccl::CameraType::CAMERA_PANORAMA)
		{
			switch(m_cclScene->camera->get_panorama_type())
			{
			case ccl::PanoramaType::PANORAMA_EQUIRECTANGULAR:
			case ccl::PanoramaType::PANORAMA_FISHEYE_EQUIDISTANT:
				flipHorizontally = false; // I have no idea why some types have to be flipped and others don't
				break;
			}
		}
		m_tileManager.SetFlipImage(flipHorizontally,true);
		m_tileManager.SetExposure(sceneInfo.exposure);
	}

	std::vector<std::pair<std::string,uimg::Format>> passes;
	passes.reserve(m_outputs.size());
	for(auto &pair : m_outputs)
		passes.push_back({pair.first,uimg::Format::RGBA32});
	auto displayDriver = std::make_unique<DisplayDriver>(m_tileManager,cam.GetWidth(),cam.GetHeight());
	m_displayDriver = displayDriver.get();
	auto outputDriver = std::make_unique<OutputDriver>(passes,cam.GetWidth(),cam.GetHeight());
	m_outputDriver = outputDriver.get();
	m_cclSession->set_display_driver(std::move(displayDriver));
	m_cclSession->set_output_driver(std::move(outputDriver));

	//

#if 0
	{
		Options opts {};

		opts.width = m_cclScene->camera->get_full_width();
		opts.height = m_cclScene->camera->get_full_height();
		opts.filepath = "E:/projects/cycles/examples/scene_monkey.xml";
		opts.output_filepath = "E:/projects/cycles/examples/scene_monkey.png";
		opts.session = NULL;
		opts.quiet = false;
		opts.session_params.use_auto_tile = false;
		opts.session_params.tile_size = 16;
		opts.session_params.samples = 20;

		
			auto &options = opts;
			 options.output_pass = "combined";
			 options.session = m_cclSession.get();//new ccl::Session(options.session_params, options.scene_params);
			 options.scene = options.session->scene;

		m_cclSession->set_output_driver(std::make_unique<COIIOOutputDriver>(
			"E:/projects/cycles/examples/scene_monkey.png", "combined", session_print));
		m_cclSession->set_display_driver(std::make_unique<DisplayDriver>(opts.width,opts.height));

		m_cclSession->progress.set_update_callback([]() {});
	  
		/* add pass for output. */
		//ccl::Pass *pass = opts.scene->create_node<ccl::Pass>();
		//pass->set_name(ccl::ustring(opts.output_pass.c_str()));
		//pass->set_type(ccl::PASS_COMBINED);
		
		//options.session->reset(options.session_params, session_buffer_params(options));
		m_cclSession->reset(m_cclSession->params,bufferParams);
		m_cclSession->start();

		while(m_cclSession->progress.get_progress() < 1.f)
			;
		m_cclSession->draw();
		m_cclSession = nullptr;
		return false;
	}
#endif
	return true;
}

void unirender::cycles::Renderer::InitializePassShaders(const std::function<std::shared_ptr<GroupNodeDesc>(const Shader&)> &fGetPassDesc)
{
	std::unordered_map<Shader*,std::shared_ptr<CCLShader>> shaderCache;
	for(auto &chunk : m_renderData.modelCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
		{
			auto &mesh = o->GetMesh();
			auto *cclMesh = FindCclMesh(mesh);
			if(cclMesh == nullptr)
				continue;
			auto &subMeshShaders = mesh.GetSubMeshShaders();
			auto numShaders = subMeshShaders.size();
			std::unordered_map<uint32_t,uint32_t> oldShaderIndexToNewIndex {};
			for(auto i=decltype(numShaders){0u};i<numShaders;++i)
			{
				auto &shader = subMeshShaders.at(i);
				auto albedoPass = fGetPassDesc(*shader);
				if(albedoPass == nullptr)
					continue;

				std::shared_ptr<CCLShader> cclShader = nullptr;
				auto it = shaderCache.find(shader.get());
				if(it != shaderCache.end())
					cclShader = it->second;
				else
				{
					cclShader = CCLShader::Create(*this,*albedoPass);
					cclShader->Finalize(*m_scene);
					shaderCache[shader.get()] = cclShader;
				}
				cclMesh->get_used_shaders()[i] = **cclShader;
			}
			cclMesh->tag_update(m_cclScene,false);
		}
	}
}

void unirender::cycles::Renderer::InitializeAlbedoPass(bool reloadShaders)
{
	auto bufferParams = GetBufferParameters();
	uint32_t sampleCount = 1;
	auto &createInfo = m_scene->GetCreateInfo();
	if(createInfo.progressive)
	{
		// TODO: We should only need one sample, but for whatever reason some tiles will not get rendered properly if the sample count is too low. The reason for this is unknown
		// UPDATE: This should be fixed now, so this line should no longer be needed!
		sampleCount = 4;
	}
	m_cclSession->params.samples = sampleCount;
	m_cclSession->reset(m_cclSession->params,bufferParams); // We only need the normals and albedo colors for the first sample

	m_cclScene->lights.clear();

	if(reloadShaders == false)
		return;
	InitializePassShaders([](const Shader &shader) -> std::shared_ptr<GroupNodeDesc> {return shader.albedoPass;});
#if 0
	// Note: For denoising the scene has to be rendered three times:
	// 1) With lighting
	// 2) Albedo colors only
	// 3) Normals only
	// However, Cycles doesn't allow rendering to multiple outputs at once, so we
	// need three separate render passes. For the additional render passes
	// we have to replace the shaders, which is also impossible to do with Cycles.
	// Instead, we have to create an additional set of shaders for each object and
	// re-assign the shader indices of the mesh.
	std::unordered_map<Shader*,std::shared_ptr<CCLShader>> shaderCache;
	for(auto &chunk : m_renderData.modelCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
		{
			auto &mesh = o->GetMesh();
			auto &subMeshShaders = mesh.GetSubMeshShaders();
			auto numShaders = subMeshShaders.size();
			std::unordered_map<uint32_t,uint32_t> oldShaderIndexToNewIndex {};
			for(auto i=decltype(numShaders){0u};i<numShaders;++i)
			{
				auto &shader = subMeshShaders.at(i);
				auto &albedoPass = shader->albedoPass;
				if(albedoPass == nullptr)
					continue;

				std::shared_ptr<CCLShader> cclShader = nullptr;
				auto it = shaderCache.find(shader.get());
				if(it != shaderCache.end())
					cclShader = it->second;
				else
				{
					cclShader = CCLShader::Create(*this,*albedoPass);
					cclShader->Finalize(*this);
					shaderCache[shader.get()] = cclShader;
				}

				if(mesh->used_shaders.size() == mesh->used_shaders.capacity())
					mesh->used_shaders.reserve(mesh->used_shaders.size() *1.1 +50);
				mesh->used_shaders.push_back(**cclShader);
				oldShaderIndexToNewIndex[i] = mesh->used_shaders.size() -1;
			}
			auto &origShaderIndexTable = mesh.GetOriginalShaderIndexTable();
			if(origShaderIndexTable.empty())
			{
				// Note: The albedo and normal pass need to overwrite the original shaders for each mesh.
				// We keep a table with the original shader indices so the next pass (usually the normal pass)
				// has a reference to the original shader.

				// TODO: There's no need to store ALL shader indices, this can be optimized!
				origShaderIndexTable.resize(mesh->shader.size());
				for(auto i=decltype(mesh->shader.size()){0};i<mesh->shader.size();++i)
					origShaderIndexTable[i] = mesh->shader[i];
			}
			for(auto i=decltype(mesh->shader.size()){0};i<mesh->shader.size();++i)
			{
				auto &shaderIdx = mesh->shader[i];
				auto it = oldShaderIndexToNewIndex.find(shaderIdx);
				if(it == oldShaderIndexToNewIndex.end())
					continue;
				auto oldShaderIdx = shaderIdx;
				shaderIdx = it->second;
			}
			mesh->tag_update(&m_cclScene,false);
		}
	}
#endif
}

void unirender::cycles::Renderer::InitializeNormalPass(bool reloadShaders)
{
	// Also see unirender::Scene::CreateShader
	auto bufferParams = GetBufferParameters();
	uint32_t sampleCount = 1;
	auto &createInfo = m_scene->GetCreateInfo();
	if(createInfo.progressive)
	{
		// TODO: We should only need one sample, but for whatever reason some tiles will not get rendered properly if the sample count is too low. The reason for this is unknown
		// UPDATE: This should be fixed now, so this line should no longer be needed!
		sampleCount = 4;
	}
	m_cclSession->params.samples = sampleCount;
	m_cclSession->reset(m_cclSession->params,bufferParams); // We only need the normals and albedo colors for the first sample

	// Disable the sky (by making it black)
	auto shader = unirender::GroupNodeDesc::Create(m_scene->GetShaderNodeManager());

	auto &nodeOutput = shader->AddNode(NODE_OUTPUT);
	auto &nodeBg = shader->AddNode(NODE_BACKGROUND_SHADER);
	nodeBg.SetProperty(nodes::background_shader::IN_STRENGTH,0.f);

	auto col = shader->CombineRGB(0.f,0.f,0.f);
	shader->Link(col,nodeBg.GetInputSocket(nodes::background_shader::IN_COLOR));
	shader->Link(nodeBg,nodes::background_shader::OUT_BACKGROUND,nodeOutput,nodes::output::IN_SURFACE);

	auto cclShader = CCLShader::Create(*this,*shader);
	cclShader->Finalize(*m_scene);
	m_cclScene->default_background = **cclShader;
	(*cclShader)->tag_update(m_cclScene);

	if(reloadShaders == false)
		return;
	InitializePassShaders([](const Shader &shader) -> std::shared_ptr<GroupNodeDesc> {return shader.normalPass;});
#if 0
	// Note: For denoising the scene has to be rendered three times:
	// 1) With lighting
	// 2) Albedo colors only
	// 3) Normals only
	// However, Cycles doesn't allow rendering to multiple outputs at once, so we
	// need three separate render passes. For the additional render passes
	// we have to replace the shaders, which is also impossible to do with Cycles.
	// Instead, we have to create an additional set of shaders for each object and
	// re-assign the shader indices of the mesh.
	for(auto &chunk : m_renderData.modelCache->GetChunks())
	{
		for(auto &o : chunk.GetObjects())
		{
			auto &mesh = o->GetMesh();
			auto &subMeshShaders = mesh.GetSubMeshShaders();
			auto numShaders = subMeshShaders.size();
			std::unordered_map<uint32_t,uint32_t> oldShaderIndexToNewIndex {};
			for(auto i=decltype(numShaders){0u};i<numShaders;++i)
			{
				auto &shader = subMeshShaders.at(i);
				if(shader->normalPass == nullptr)
					continue;
				auto cclShader = CCLShader::Create(*this,*shader->normalPass);
				cclShader->Finalize(*this);

				if(mesh->used_shaders.size() == mesh->used_shaders.capacity())
					mesh->used_shaders.reserve(mesh->used_shaders.size() *1.1 +50);
				mesh->used_shaders.push_back(**cclShader);
				oldShaderIndexToNewIndex[i] = mesh->used_shaders.size() -1;
			}
			auto &origShaderIndexTable = mesh.GetOriginalShaderIndexTable();
			for(auto i=decltype(mesh->shader.size()){0};i<mesh->shader.size();++i)
			{
				auto shaderIdx = (i < origShaderIndexTable.size()) ? origShaderIndexTable.at(i) : mesh->shader[i];
				auto it = oldShaderIndexToNewIndex.find(shaderIdx);
				if(it == oldShaderIndexToNewIndex.end())
					continue;
				mesh->shader[i] = it->second;
			}
			mesh->tag_update(&m_cclScene,false);
		}
	}
#endif
}

std::shared_ptr<unirender::CCLShader> unirender::cycles::Renderer::GetCachedShader(const GroupNodeDesc &desc) const
{
	auto it = m_shaderCache.find(&desc);
	if(it == m_shaderCache.end())
		return nullptr;
	auto idx = it->second;
	return m_cclShaders.at(idx);
}

void unirender::cycles::Renderer::AddShader(CCLShader &shader,const GroupNodeDesc *optDesc)
{
	m_cclShaders.push_back(shader.shared_from_this());
	if(optDesc)
		m_shaderCache[optDesc] = m_cclShaders.size() -1;
}

void unirender::cycles::Renderer::Reset()
{
	m_restartState = 1;
	SetCancelled("Cancelled by user");
	m_cclSession->wait();
	m_cclSession->progress.reset();
}
void unirender::cycles::Renderer::Restart()
{
	auto &createInfo = m_scene->GetCreateInfo();
	if(createInfo.progressive)
		m_tileManager.Reload(false);

	m_cclSession->start();
	m_restartState = 2;
}
std::optional<std::string> unirender::cycles::Renderer::SaveRenderPreview(const std::string &path,std::string &outErr) const
{
	outErr = "Saving render preview not implemented for cycles.";
	return {};
}

void unirender::cycles::Renderer::AddSkybox(const std::string &texture)
{
	if(umath::is_flag_set(m_stateFlags,StateFlags::SkyInitialized))
		return;
	umath::set_flag(m_stateFlags,StateFlags::SkyInitialized);

	if(m_renderMode == Scene::RenderMode::SceneDepth)
	{
		auto desc = unirender::GroupNodeDesc::Create(m_scene->GetShaderNodeManager());
		auto &nodeOutput = desc->AddNode(NODE_OUTPUT);
		auto &nodeBg = desc->AddNode(NODE_BACKGROUND_SHADER);
		nodeBg.SetProperty(nodes::background_shader::IN_STRENGTH,1'000.f);

		auto col = desc->CombineRGB(1.f,1.f,1.f);
		desc->Link(col,nodeBg.GetInputSocket(nodes::background_shader::IN_COLOR));
		desc->Link(nodeBg,nodes::background_shader::OUT_BACKGROUND,nodeOutput,nodes::output::IN_SURFACE);
		
		AddShader(*CCLShader::Create(*this,*m_cclScene->default_background,*desc));
		return;
	}

	auto &sceneInfo = m_scene->GetSceneInfo();
	auto skyTex = (sceneInfo.sky.empty() == false) ? sceneInfo.sky : texture;

	// Note: m_sky can be absolute or relative path
	auto absPath = Scene::GetAbsSkyPath(skyTex);
	if(absPath.has_value() == false)
		return;

	// Setup the skybox as a background shader
	auto desc = unirender::GroupNodeDesc::Create(m_scene->GetShaderNodeManager());
	auto &nodeOutput = desc->AddNode(NODE_OUTPUT);
	auto &nodeBg = desc->AddNode(NODE_BACKGROUND_SHADER);
	nodeBg.SetProperty(nodes::background_shader::IN_STRENGTH,sceneInfo.skyStrength);
	
	auto &nodeTex = desc->AddImageTextureNode(*absPath,TextureType::EquirectangularImage);
	desc->Link(nodeTex,nodes::environment_texture::OUT_COLOR,nodeBg,nodes::background_shader::IN_COLOR);
	desc->Link(nodeBg,nodes::background_shader::OUT_BACKGROUND,nodeOutput,nodes::output::IN_SURFACE);

	auto skyAngles = sceneInfo.skyAngles;
	// skyAngles.p -= 90.f;
	skyAngles = {
		-skyAngles.p,
		skyAngles.r,
		-skyAngles.y
	};

	auto &nodeTexCoord = desc->AddNode(NODE_TEXTURE_COORDINATE);
	auto &nodeMapping = desc->AddNode(NODE_MAPPING);
	nodeMapping.SetProperty(nodes::mapping::IN_TYPE,ccl::NodeMappingType::NODE_MAPPING_TYPE_POINT);
	nodeMapping.SetProperty(nodes::mapping::IN_ROTATION,skyAngles);
	desc->Link(nodeTexCoord,nodes::texture_coordinate::OUT_GENERATED,nodeMapping,nodes::mapping::IN_VECTOR);

	desc->Link(nodeMapping,nodes::mapping::OUT_VECTOR,nodeTex,nodes::environment_texture::IN_VECTOR);
	AddShader(*CCLShader::Create(*this,*m_cclScene->default_background,*desc));

	// Add the light source for the background
	auto *light = new ccl::Light{}; // Object will be removed automatically by cycles
	light->set_tfm(ccl::transform_identity());

	m_cclScene->lights.push_back(light);
	light->set_light_type(ccl::LightType::LIGHT_BACKGROUND);
	light->set_map_resolution(2'048);
	light->set_shader(m_cclScene->default_background);
	light->set_use_mis(true);
	light->set_max_bounces(1'024);
	light->tag_update(m_cclScene);
}

static void validate_session(ccl::Scene &scene)
{
	for(auto *shader : scene.shaders)
	{
		if(shader->graph == nullptr)
			throw std::logic_error{"Found shader with invalid graph!"};
	}
}

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

			if(Scene::IsRenderSceneMode(m_renderMode))
			{
				auto &cam = m_scene->GetCamera();
				auto stereoscopic = cam.IsStereoscopic();
				if(stereoscopic)
					m_cclScene->camera->set_stereo_eye(ccl::Camera::StereoEye::STEREO_LEFT);
				ImageRenderStage initialRenderStage;
				switch(m_renderMode)
				{
				case Scene::RenderMode::RenderImage:
					initialRenderStage = ImageRenderStage::Lighting;
					break;
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
					throw std::invalid_argument{"Invalid render mode " +std::to_string(umath::to_integral(m_renderMode))};
				}
				StartNextRenderStage(worker,initialRenderStage,stereoscopic ? StereoEye::Left : StereoEye::None);
				worker.Start();
			}
			else
			{
				StartNextRenderStage(worker,ImageRenderStage::Bake,StereoEye::None);
				worker.Start();
			}
			return RenderStageResult::Continue;
		});
		break;
	}
	case ImageRenderStage::Bake:
	{
		StartTextureBaking(worker);
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
				resultImgBuf->ToHDR();
				// ApplyPostProcessing(*resultImageBuffer,m_renderMode);

				if(UpdateStereoEye(worker,stage,eyeStage))
				{
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

				if(UpdateStereoEye(worker,stage,eyeStage))
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

				if(UpdateStereoEye(worker,stage,eyeStage))
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
		if(umath::min(m_cclSession->progress.get_progress(),1.0) == 1.0)
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

	if(m_cclSession == nullptr)
		return;
	m_cclSession = nullptr;
}

#include <util_image.hpp>
#include <fsys/filesystem.h>
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
	if(m_cclSession && m_scene->IsRenderSceneMode(m_scene->GetRenderMode()) && umath::is_flag_set(m_stateFlags,StateFlags::RenderingStarted))
		GetResultImageBuffer(OUTPUT_COLOR) = GetOutputDriver()->GetImageBuffer(OUTPUT_COLOR);
	CloseCyclesScene();
}

void unirender::cycles::Renderer::CloseRenderScene() {CloseCyclesScene();}

void unirender::cycles::Renderer::FinalizeImage(uimg::ImageBuffer &imgBuf,StereoEye eyeStage)
{
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
	integrator.set_sampling_pattern(ccl::SamplingPattern::SAMPLING_PATTERN_SOBOL);

	auto apiData = GetApiData();
	auto useAdaptiveSampling = true;
	apiData.GetFromPath("cycles/useAdaptiveSampling")(useAdaptiveSampling);
	if(useAdaptiveSampling)
	{
		auto adaptiveSamplingThreshold = 0.01f;
		uint32_t minSamples = 0;
		apiData.GetFromPath("cycles/adaptiveSamplingThreshold")(adaptiveSamplingThreshold);
		apiData.GetFromPath("cycles/adaptiveMinSamples")(minSamples);
		integrator.set_use_adaptive_sampling(true);
		integrator.set_adaptive_threshold(adaptiveSamplingThreshold);
		integrator.set_adaptive_min_samples(0);
	}

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
	integrator.tag_modified();

	// Film
	auto &film = *scene.film;
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

util::ParallelJob<std::shared_ptr<uimg::ImageBuffer>> unirender::cycles::Renderer::StartRender()
{
	auto job = util::create_parallel_job<RenderWorker>(*this);
	auto &worker = static_cast<RenderWorker&>(job.GetWorker());
	StartNextRenderStage(worker,ImageRenderStage::InitializeScene,StereoEye::None);
	return job;
}
#if 0
void unirender::cycles::Renderer::UpdateRenderTile(unirender::TileManager &tileManager,const ccl::RenderTile &tile,bool param)
{
	auto tileSize = tileManager.GetTileSize();
	auto numTilesPerAxis = tileManager.GetTilesPerAxisCount();

	assert((tile.x %tileSize.x) == 0 && (tile.y %tileSize.y) == 0);
	if((tile.x %tileSize.x) != 0 || (tile.y %tileSize.y) != 0)
		throw std::invalid_argument{"Unexpected tile size"};
	auto tileIndex = tile.x /tileSize.x +(tile.y /tileSize.y) *numTilesPerAxis.x;
	unirender::TileManager::TileData data {};
	data.x = tile.x;
	data.y = tile.y;
	data.index = tileIndex; // tile.tile_index; // tile_index doesn't match expected tile index in some cases?
	data.sample = tile.sample;
	data.data.resize(tile.w *tile.h *sizeof(Vector4));
	data.w = tile.w;
	data.h = tile.h;
	if(tileManager.IsCpuDevice() == false)
		tile.buffers->copy_from_device(); // TODO: Is this the right way to do this?
	tile.buffers->get_pass_rect("combined",tileManager.GetExposure(),tile.sample,4,reinterpret_cast<float*>(data.data.data()));
	// We want to minimize the overhead on this thread as much as possible (to avoid stalling Cycles), so we'll continue with post-processing on yet another thread
	auto &inputTileMutex = tileManager.GetInputTileMutex();
	auto &inputTileQueue = tileManager.GetInputTileQueue();
	inputTileMutex.lock();
		auto &inputTile = tileManager.GetInputTiles()[tileIndex];
		if(tile.sample > inputTile.sample || inputTile.sample == std::numeric_limits<decltype(inputTile.sample)>::max())
		{
			inputTile = std::move(data);
			inputTileQueue.push(tileIndex);
			tileManager.NotifyPendingWork();
		}
	inputTileMutex.unlock();
}
void unirender::cycles::Renderer::WriteRenderTile(unirender::TileManager &tileManager,const ccl::RenderTile &tile)
{
	// TODO: What's this callback for exactly?
}
#endif
extern "C" {
	bool __declspec(dllexport) create_renderer(const unirender::Scene &scene,unirender::Renderer::Flags flags,std::shared_ptr<unirender::Renderer> &outRenderer)
	{
		unirender::Scene::SetKernelPath(util::get_program_path() +"/modules/unirender/cycles");
		outRenderer = unirender::cycles::Renderer::Create(scene,flags);
		return outRenderer != nullptr;
	}
};
#pragma optimize("",on)
