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
#include "unirender/cycles/cycles_interface.hpp"
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
#include <device/device.h>
#include <util/path.h>
#include <util_image_buffer.hpp>

#ifdef _WIN32
#include <Shlobj.h>
#endif

#ifdef __linux__
#include <OpenImageIO/ustring.h>
// This fixes an odd missing symbol issue
// May have something to do with this: https://github.com/OpenImageIO/oiio/pull/1176/commits/a2ccfad7c4962a5203ea2cf755fd102b4c67f997
std::string ccl::ustring::empty_std_string;
#endif

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
	icycles::util::path_init(kernelPath.c_str(),kernelPath.c_str());

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
		putenv(("OPTIX_ROOT_DIR=" +*optixPath).data());
	}
	else
		std::cout<<"Could not find Optix SDK! Dynamic Optix kernel building will be disabled!"<<std::endl;
#ifdef ENABLE_CYCLES_LOGGING
	icycles::util::enable_logging(
		"util_raytracing",
		(cyclesPath +"/log/info.log").c_str(),
		(cyclesPath +"/log/warning.log").c_str(),
		(cyclesPath +"/log/error.log").c_str(),
		(cyclesPath +"/log/fatal.log").c_str(),
		(cyclesPath +"/log").c_str()
	);
#endif
}

static bool is_device_type_available(ccl::DeviceType type)
{
	return icycles::device::is_type_available(type);
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
std::shared_ptr<unirender::cycles::Renderer> unirender::cycles::Renderer::Create(const unirender::Scene &scene,std::string &outErr,Flags flags)
{
	auto renderer = std::shared_ptr<Renderer>{new Renderer{scene,flags}};
	renderer->m_renderMode = scene.GetRenderMode();

	auto &createInfo = scene.GetCreateInfo();
	umath::set_flag(renderer->m_stateFlags,StateFlags::ProgressiveRefine,createInfo.progressiveRefine);
	auto &sceneInfo = scene.GetSceneInfo();
	const_cast<Scene::SceneInfo&>(sceneInfo).exposure = createInfo.exposure; // TODO: This doesn't belong here!
	return renderer;
}

unirender::cycles::Renderer::Renderer(const Scene &scene,Flags flags)
	: unirender::Renderer{scene,flags}
{}
unirender::cycles::Renderer::~Renderer()
{
	FinalizeAndCloseCyclesScene();
#ifdef ENABLE_CYCLES_LOGGING
	icycles::util::flush_log();
#endif
}

icycles::SessionParams unirender::cycles::Renderer::GetSessionParameters(const unirender::Scene &scene,const ccl::DeviceInfo &devInfo) const
{
	auto &createInfo = scene.GetCreateInfo();
	icycles::SessionParams sessionParams;
	icycles::util::create_session_params(sessionParams);
	icycles::session_params::set_shadingsystem(*sessionParams,ccl::SHADINGSYSTEM_SVM);
	icycles::session_params::set_device(*sessionParams,devInfo);
	icycles::session_params::set_background(*sessionParams,!umath::is_flag_set(m_flags,Flags::EnableLiveEditing)); // Live denoising will not work for background mode
	icycles::session_params::set_use_auto_tile(*sessionParams,false); // Tile rendering is no longer relevant for Cycles X (and causes the output driver to not function properly)

	switch(m_deviceType)
	{
	case unirender::Scene::DeviceType::GPU:
		icycles::session_params::set_tile_size(*sessionParams,256);
		break;
	default:
		icycles::session_params::set_tile_size(*sessionParams,16);
		break;
	}

	if(createInfo.samples.has_value())
		icycles::session_params::set_samples(*sessionParams,*createInfo.samples);
	else
	{
		switch(m_renderMode)
		{
		case unirender::Scene::RenderMode::BakeAmbientOcclusion:
		case unirender::Scene::RenderMode::BakeNormals:
		case unirender::Scene::RenderMode::BakeDiffuseLighting:
		case unirender::Scene::RenderMode::BakeDiffuseLightingSeparate:
			icycles::session_params::set_samples(*sessionParams,1'225u);
			break;
		default:
			icycles::session_params::set_samples(*sessionParams,1'024u);
			break;
		}
	}

	if(umath::is_flag_set(m_stateFlags,StateFlags::ProgressiveRefine))
		icycles::session_params::set_samples(*sessionParams,50'000);

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
std::shared_ptr<ccl::DeviceInfo> unirender::cycles::Renderer::InitializeDevice(const unirender::Scene &scene,std::string &outErr)
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
		auto dev = icycles::device::type_from_string(deviceOverride.c_str());
		if(dev == ccl::DeviceType::DEVICE_NONE)
		{
			std::string availableTypes;
			std::vector<ccl::DeviceType> deviceTypes;
			icycles::device::available_types(deviceTypes);
			for(auto devType : deviceTypes)
			{
				if(!availableTypes.empty())
					availableTypes += ", ";
				icycles::CString strType {};
				icycles::device::string_from_type(devType,&strType.cstr,strType.len);
				availableTypes += strType;
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
	std::shared_ptr<ccl::DeviceInfo> device = {};
	std::vector<std::shared_ptr<ccl::DeviceInfo>> vDevices;
	icycles::device::get_available_devices(devices,vDevices);
	for(auto &devInfo : vDevices)
	{
		if(icycles::device_info::get_type(*devInfo) == cclDeviceType)
		{
			device = devInfo;
			break;
		}
		if(icycles::device_info::get_type(*devInfo) == ccl::DeviceType::DEVICE_CPU)
			device = devInfo; // Fallback / Default device type
	}
	
	if(!device)
	{
		icycles::CString device {};
		icycles::device::string_from_type(cclDeviceType,&device.cstr,device.len);
		outErr = "Device '" +static_cast<std::string>(device) +"' is not available!";
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
	icycles::SceneParams sceneParams;
	icycles::util::create_scene_params(sceneParams);
	icycles::scene_params::set_shadingsystem(*sceneParams,ccl::SHADINGSYSTEM_SVM);

#if 0
	// Debug values
	sceneParams.shadingsystem = ccl::SHADINGSYSTEM_SVM;
	sceneParams.bvh_type = ccl::BVH_TYPE_STATIC;
	sceneParams.use_bvh_spatial_split = false;
	sceneParams.use_bvh_compact_structure = true;
	sceneParams.use_bvh_unaligned_nodes = true;
	sceneParams.num_bvh_time_steps = 0;
	sceneParams.hair_subdivisions = 2;
	sceneParams.hair_shape = ccl::CURVE_RIBBON;
	sceneParams.texture_limit = 0;
	sceneParams.bvh_layout = ccl::BVH_LAYOUT_EMBREE;
	sceneParams.background = true;
#endif

	auto sessionParams = GetSessionParameters(scene,devInfo);
	icycles::session::create(*sessionParams,*sceneParams,m_cclSession);

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
	return (it != m_objectToCclObject.end()) ? it->second.object : nullptr;
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
	if(icycles::attribute::data_sizeof(attr) != sizeof(TCcl))
		throw std::logic_error{"Data size mismatch"};

	std::vector<TCcl> cclValues;
	cclValues.reserve(data.size());
	for(auto &v : data)
		cclValues.push_back(translate(v));

	icycles::attribute::resize(attr,cclValues.size());
	auto *ptr = icycles::attribute::data(attr);
	memcpy(ptr,cclValues.data(),cclValues.size() *sizeof(cclValues[0]));
}

template<typename T,typename TCcl>
	static void initialize_attribute(ccl::Geometry &mesh,ccl::AttributeStandard attrs,const std::vector<T> &data,const std::function<TCcl(const T&)> &translate)
{
	auto *attr = icycles::attribute_set::add(icycles::geometry::get_attributes(mesh),attrs);
	if(!attr)
		return;
	copy_vector_to_attribute(data,*attr,translate);
}

void unirender::cycles::Renderer::SyncObject(const unirender::Object &obj)
{
	auto *cclObj = icycles::object::create();
	icycles::node::set_name(*cclObj,obj.GetName().c_str());
	m_cclScene->objects.push_back(cclObj);
	auto &pose = obj.GetPose();
	auto cclPose = ToCyclesTransform(pose);
	m_objectToCclObject[&obj] = {cclObj,pose};
	m_uuidToObject[util::uuid_to_string(obj.GetUuid())] = &obj;
	icycles::object::set_tfm(*cclObj,cclPose);
	auto &mesh = obj.GetMesh();
	auto *cclMesh = FindCclMesh(mesh);
	icycles::object::set_geometry(*cclObj,cclMesh);
	// m_object.tag_update(*scene);

#ifdef ENABLE_MOTION_BLUR_TEST
	m_motionPose.SetOrigin(Vector3{100.f,100.f,100.f});
	m_object.motion.push_back_slow(Scene::ToCyclesTransform(GetMotionPose()));
#endif
	auto &usedShaders = icycles::geometry::get_used_shaders(*cclMesh);
	for(auto &set : mesh.GetHairStrandDataSets())
	{
		if(set.shaderIndex >= usedShaders.size())
			continue;
		auto *shader = usedShaders[set.shaderIndex];

		auto *cclHair = icycles::hair::create();
		m_cclScene->geometry.push_back(cclHair);

		auto numHair = set.strandData.hairSegments.size();
		uint32_t pointOffset = 0;
		icycles::hair::reserve_curves(*cclHair,numHair,set.strandData.points.size());
		std::vector<Vector2> testUv;
		testUv.reserve(numHair);
		for(auto i=decltype(numHair){0u};i<numHair;++i)
		{
			auto numSegments = set.strandData.hairSegments[i];
			auto numPoints = numSegments +1;
			icycles::hair::add_curve(*cclHair,pointOffset,0 /* shader */);
			for(auto j=decltype(numPoints){0u};j<numPoints;++j)
			{
				auto p = pose *set.strandData.points[pointOffset +j];
				auto thickness = set.strandData.thicknessData[pointOffset +j];
				icycles::hair::add_curve_key(*cclHair,ToCyclesPosition(p),thickness);
			}
			testUv.push_back(set.strandData.uvs[pointOffset]);

			pointOffset += numPoints;
		}

		initialize_attribute<Vector2,ccl::float2>(*cclHair,ccl::ATTR_STD_UV,testUv,[](const Vector2 &v) -> ccl::float2 {return ToCyclesUV(v);});

		ccl::array<ccl::Node*> shaders;
		shaders.resize(1);
		shaders[0] = shader;
		icycles::geometry::set_used_shaders(*cclHair,shaders);

		//attr_uv = hair->attributes.add(name, TypeFloat2, ATTR_ELEMENT_CURVE);

		auto *cclObj = icycles::object::create();
		m_cclScene->objects.push_back(cclObj);
		//cclObj->set_tfm(ToCyclesTransform(obj.GetPose()));
		icycles::object::set_geometry(*cclObj,cclHair);
	}
}

static ccl::ShaderInput *find_input_socket(ccl::ShaderNode &node,const char *strInput)
{
	auto n = icycles::shader_node::get_input_count(node);
	for(auto i=decltype(n){0u};i<n;++i)
	{
		auto &input = *icycles::shader_node::get_input(node,i);
		auto &socketType = icycles::shader_input::get_socket_type(input);
		icycles::CString socketTypeName {};
		icycles::socket_type::get_name(socketType,&socketTypeName.cstr,socketTypeName.len);
		if(icycles::util::string_iequals(socketTypeName.cstr,strInput))
			return &input;
	}
	return nullptr;
}
static const ccl::SocketType *find_type_input(ccl::ShaderNode &node,const char *strInput)
{
	auto n = icycles::node_type::get_input_count(*node.type);
	for(auto i=decltype(n){0u};i<n;++i)
	{
		auto &socketType = *icycles::node_type::get_input(*node.type,i);
		icycles::CString socketTypeName {};
		icycles::socket_type::get_name(socketType,&socketTypeName.cstr,socketTypeName.len);
		if(icycles::util::string_iequals(socketTypeName.cstr,strInput))
			return &socketType;
	}
	return nullptr;
}
static ccl::ShaderOutput *find_output_socket(ccl::ShaderNode &node,const char *strOutput)
{
	auto n = icycles::shader_node::get_output_count(node);
	for(auto i=decltype(n){0u};i<n;++i)
	{
		auto &output = *icycles::shader_node::get_output(node,i);
		auto &socketType = icycles::shader_output::get_socket_type(output);
		icycles::CString socketTypeName {};
		icycles::socket_type::get_name(socketType,&socketTypeName.cstr,socketTypeName.len);
		if(icycles::util::string_iequals(socketTypeName.cstr,strOutput))
			return &output;
	}
	return nullptr;
}
void unirender::cycles::Renderer::SyncMesh(const unirender::Mesh &mesh)
{
	auto *cclMesh = icycles::mesh::create();
	m_cclScene->geometry.push_back(cclMesh);
	m_meshToCcclMesh[&mesh] = cclMesh;
	m_cclMeshToMesh[cclMesh] = &mesh;

	icycles::node::set_name(*cclMesh,mesh.GetName().c_str());
	icycles::mesh::reserve_mesh(*cclMesh,mesh.GetVertexCount(),mesh.GetTriangleCount());
	for(auto &v : mesh.GetVertices())
		icycles::mesh::add_vertex(*cclMesh,ToCyclesPosition(v));
	auto &tris = mesh.GetTriangles();
	auto &shaderIds = mesh.GetShaders();
	auto &smooth = mesh.GetSmooth();
	auto ntris = tris.size();
	for(auto i=decltype(ntris){0u};i<ntris;i+=3)
		icycles::mesh::add_triangle(*cclMesh,tris[i],tris[i +1],tris[i +2],shaderIds[i /3],smooth[i /3]);

	auto fToFloat4 = [](const ccl::float3 &v) -> ccl::float4 {return ccl::float4{v.x,v.y,v.z,0.f};};
	initialize_attribute<Vector3,ccl::float4>(*cclMesh,ccl::ATTR_STD_VERTEX_NORMAL,mesh.GetVertexNormals(),[&fToFloat4](const Vector3 &v) -> ccl::float4 {return fToFloat4(ToCyclesNormal(v));});
	initialize_attribute<Vector2,ccl::float2>(*cclMesh,ccl::ATTR_STD_UV,mesh.GetUvs(),[](const Vector2 &v) -> ccl::float2 {return ToCyclesUV(v);});
	initialize_attribute<Vector3,ccl::float3>(*cclMesh,ccl::ATTR_STD_UV_TANGENT,mesh.GetUvTangents(),[](const Vector3 &v) -> ccl::float3 {return ToCyclesNormal(v);});
	initialize_attribute<float,float>(*cclMesh,ccl::ATTR_STD_UV_TANGENT_SIGN,mesh.GetUvTangentSigns(),[](const float &v) -> float {return v;});

	auto *attrT = icycles::attribute_set::add(icycles::geometry::get_attributes(*cclMesh),ccl::ATTR_STD_UV_TANGENT);
	if(attrT)
		icycles::attribute::set_name(*attrT,("orco" +Mesh::TANGENT_POSTFIX).c_str());

	auto *attrTS = icycles::attribute_set::add(icycles::geometry::get_attributes(*cclMesh),ccl::ATTR_STD_UV_TANGENT_SIGN);
	if(attrTS)
		icycles::attribute::set_name(*attrTS,("orco" +Mesh::TANGENT_SIGN_POSTIFX).c_str());

	if(mesh.HasAlphas())
	{
		auto &alphas = mesh.GetAlphas();
		icycles::attribute_set::add(icycles::geometry::get_attributes(*cclMesh),ALPHA_ATTRIBUTE_TYPE);
		initialize_attribute<float,float>(*cclMesh,ALPHA_ATTRIBUTE_TYPE,*alphas,[](const float &v) -> float {return v;});
	}

	auto &shaders = mesh.GetSubMeshShaders();
	ccl::array<ccl::Node*> usedShaders;
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
	auto usedShadersCpy = usedShaders;
	icycles::geometry::set_used_shaders(*cclMesh,usedShadersCpy);

	// TODO: We should be using the tangent values from m_tangents / m_tangentSigns
	// but their coordinate system needs to be converted for Cycles.
	// For now we'll just re-compute the tangents here.
	compute_tangents(cclMesh,true,true);
	
	if(icycles::geometry::need_attribute(*cclMesh,m_cclScene,ccl::ATTR_STD_GENERATED))
	{
		auto *attr = icycles::attribute_set::add(icycles::geometry::get_attributes(*cclMesh),ccl::ATTR_STD_GENERATED);
		memcpy(icycles::attribute::data_float3(*attr), icycles::mesh::get_verts(*cclMesh).data(), sizeof(ccl::float3) *icycles::mesh::get_verts(*cclMesh).size());
	}
}

void unirender::cycles::Renderer::SyncCamera(const unirender::Camera &cam,bool update)
{
	auto &cclCam = *(*this)->camera;

	switch(cam.GetType())
	{
	case unirender::Camera::CameraType::Perspective:
		icycles::camera::set_camera_type(cclCam,ccl::CameraType::CAMERA_PERSPECTIVE);
		break;
	case unirender::Camera::CameraType::Orthographic:
		icycles::camera::set_camera_type(cclCam,ccl::CameraType::CAMERA_ORTHOGRAPHIC);
		break;
	case unirender::Camera::CameraType::Panorama:
		icycles::camera::set_camera_type(cclCam,ccl::CameraType::CAMERA_PANORAMA);
		break;
	}

	switch(cam.GetPanoramaType())
	{
	case unirender::Camera::PanoramaType::Equirectangular:
		icycles::camera::set_panorama_type(cclCam,ccl::PanoramaType::PANORAMA_EQUIRECTANGULAR);
		break;
	case unirender::Camera::PanoramaType::FisheyeEquidistant:
		icycles::camera::set_panorama_type(cclCam,ccl::PanoramaType::PANORAMA_FISHEYE_EQUIDISTANT);
		break;
	case unirender::Camera::PanoramaType::FisheyeEquisolid:
		icycles::camera::set_panorama_type(cclCam,ccl::PanoramaType::PANORAMA_FISHEYE_EQUISOLID);
		break;
	case unirender::Camera::PanoramaType::Mirrorball:
		icycles::camera::set_panorama_type(cclCam,ccl::PanoramaType::PANORAMA_MIRRORBALL);
		break;
	}

	icycles::camera::set_full_width(cclCam,cam.GetWidth());
	icycles::camera::set_full_height(cclCam,cam.GetHeight());
	icycles::camera::set_nearclip(cclCam,cam.GetNearZ());
	icycles::camera::set_farclip(cclCam,cam.GetFarZ());
	icycles::camera::set_fov(cclCam,umath::deg_to_rad(cam.GetFov()));
	icycles::camera::set_focaldistance(cclCam,cam.GetFocalDistance());
	icycles::camera::set_aperturesize(cclCam,cam.GetApertureSize());
	icycles::camera::set_aperture_ratio(cclCam,cam.GetApertureRatio());
	icycles::camera::set_blades(cclCam,cam.GetBladeCount());
	icycles::camera::set_bladesrotation(cclCam,umath::deg_to_rad(cam.GetBladesRotation()));
	icycles::camera::set_interocular_distance(cclCam,units::convert<units::length::millimeter,units::length::meter>(cam.GetInterocularDistance()));
	icycles::camera::set_longitude_max(cclCam,umath::deg_to_rad(cam.GetLongitudeMax()));
	icycles::camera::set_longitude_min(cclCam,umath::deg_to_rad(cam.GetLongitudeMin()));
	icycles::camera::set_latitude_max(cclCam,umath::deg_to_rad(cam.GetLatitudeMax()));
	icycles::camera::set_latitude_min(cclCam,umath::deg_to_rad(cam.GetLatitudeMin()));
	icycles::camera::set_use_spherical_stereo(cclCam,cam.IsStereoscopic());

#ifdef ENABLE_MOTION_BLUR_TEST
	SetShutterTime(1.f);
#endif

	if(cam.IsDofEnabled() == false)
		icycles::camera::set_aperturesize(cclCam,0.f);
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
			icycles::camera::set_fisheye_lens(cclCam,10.5f);
			icycles::camera::set_fisheye_fov(cclCam,180.f);
			// No break is intentional!
		default:
			rot *= uquat::create(EulerAngles{-90.f,-90.f,0.f});
			break;
		}
		pose.SetRotation(rot);
	}

	icycles::camera::set_matrix(cclCam,ToCyclesTransform(pose,true));
	icycles::camera::compute_auto_viewplane(cclCam);
	
	if(update)
		return;
	//
	std::cout<<"Camera settings:"<<std::endl;
	std::cout<<"Width: "<<icycles::camera::get_full_width(cclCam)<<std::endl;
	std::cout<<"Height: "<<icycles::camera::get_full_height(cclCam)<<std::endl;
	std::cout<<"NearZ: "<<icycles::camera::get_nearclip(cclCam)<<std::endl;
	std::cout<<"FarZ: "<<icycles::camera::get_farclip(cclCam)<<std::endl;
	std::cout<<"FOV: "<<umath::rad_to_deg(icycles::camera::get_fov(cclCam))<<std::endl;
	std::cout<<"Focal Distance: "<<icycles::camera::get_focaldistance(cclCam)<<std::endl;
	std::cout<<"Aperture Size: "<<icycles::camera::get_aperturesize(cclCam)<<std::endl;
	std::cout<<"Aperture Ratio: "<<icycles::camera::get_aperture_ratio(cclCam)<<std::endl;
	std::cout<<"Blades: "<<icycles::camera::get_blades(cclCam)<<std::endl;
	std::cout<<"Blades Rotation: "<<icycles::camera::get_bladesrotation(cclCam)<<std::endl;
	std::cout<<"Interocular Distance: "<<icycles::camera::get_interocular_distance(cclCam)<<std::endl;
	std::cout<<"Longitude Max: "<<icycles::camera::get_longitude_max(cclCam)<<std::endl;
	std::cout<<"Longitude Min: "<<icycles::camera::get_longitude_min(cclCam)<<std::endl;
	std::cout<<"Latitude Max: "<<icycles::camera::get_latitude_max(cclCam)<<std::endl;
	std::cout<<"Latitude Min: "<<icycles::camera::get_latitude_min(cclCam)<<std::endl;
	std::cout<<"Use Spherical Stereo: "<<icycles::camera::get_use_spherical_stereo(cclCam)<<std::endl;
	std::cout<<"Matrix: ";
	auto first = true;
	for(uint8_t i=0;i<3;++i)
	{
		for(uint8_t j=0;j<4;++j)
		{
			auto v = icycles::camera::get_matrix(cclCam)[i][j];
			if(first)
				first = false;
			else
				std::cout<<",";
			std::cout<<v;
		}
	}
	std::cout<<std::endl;
	//

#if 0
	// Debug values
	cclCam.viewplane.left = -1.77777779;
	cclCam.viewplane.right = 1.77777779;
	cclCam.viewplane.bottom = -1.00000000;
	cclCam.viewplane.top = 1.00000000;

	/* clipping distances */
	cclCam.set_nearclip(0.100000001);
	cclCam.set_farclip(100.000000);

	/* type */
	cclCam.set_camera_type(ccl::CAMERA_PERSPECTIVE);

	/* panorama */
	cclCam.set_panorama_type(ccl::PANORAMA_FISHEYE_EQUISOLID);
	cclCam.set_fisheye_fov(3.14159274);
	cclCam.set_fisheye_lens(10.5000000);
	cclCam.set_latitude_min(-1.57079637);
	cclCam.set_latitude_max(1.57079637);

	cclCam.set_fisheye_polynomial_k0(-1.17351437e-05);
	cclCam.set_fisheye_polynomial_k1(-0.0199887361);
	cclCam.set_fisheye_polynomial_k2(-3.35253230e-06);
	cclCam.set_fisheye_polynomial_k3(3.09927532e-06);
	cclCam.set_fisheye_polynomial_k4(-2.60646473e-08);

	cclCam.set_longitude_min(-3.14159274);
	cclCam.set_longitude_max(3.14159274);

	/* panorama stereo */
	cclCam.set_interocular_distance(0.0649999976);
	cclCam.set_convergence_distance(1.94999993);
	cclCam.set_use_spherical_stereo(false);

	/*if (cclCam.get_use_spherical_stereo()) {
	if (strcmp(viewname, "left") == 0)
	cclCam.set_stereo_eye(Camera::STEREO_LEFT);
	else if (strcmp(viewname, "right") == 0)
	cclCam.set_stereo_eye(Camera::STEREO_RIGHT);
	else
	cclCam.set_stereo_eye(Camera::STEREO_NONE);
	}*/

	cclCam.set_use_pole_merge(false);
	cclCam.set_pole_merge_angle_from(1.04719758);
	cclCam.set_pole_merge_angle_to(1.30899692);

	/* anamorphic lens bokeh */
	cclCam.set_aperture_ratio(1.00000000);

	/* perspective */
	cclCam.set_fov(2.0f * atanf((0.5f * 36.0000000) / 50.0000000 / 1.77777779));
	cclCam.set_focaldistance(0.00000000);
	cclCam.set_aperturesize(0.00000000);
	cclCam.set_blades(0);
	cclCam.set_bladesrotation(0.00000000);

	/* transform */

	//array<Transform> motion;
	//motion.resize(bcclCam.motion_steps, cclCam.get_matrix());
	//cclCam.set_motion(motion);
	cclCam.set_use_perspective_motion(false);

	cclCam.set_shuttertime(0.500000000);
	cclCam.set_fov_pre(cclCam.get_fov());
	cclCam.set_fov_post(cclCam.get_fov());
	cclCam.set_motion_position(ccl::MOTION_POSITION_CENTER);

	cclCam.set_rolling_shutter_type(ccl::Camera::ROLLING_SHUTTER_NONE);
	cclCam.set_rolling_shutter_duration(0.100000001);

	//cclCam.set_shutter_curve(bcclCam.shutter_curve);

	/* border */
	cclCam.set_border_left(0.00000000);
	cclCam.set_border_right(1);
	cclCam.set_border_top(1);
	cclCam.set_border_bottom(0);

	cclCam.set_viewport_camera_border_left(0);
	cclCam.set_viewport_camera_border_right(1);
	cclCam.set_viewport_camera_border_top(1);
	cclCam.set_viewport_camera_border_bottom(0);

	//bcclCam.offscreen_dicing_scale = RNA_float_get(cscene, "offscreen_dicing_scale");
	cclCam.set_offscreen_dicing_scale(1);
#endif

	auto &x = *this;
	auto &y = *x;
	std::cout<<"Ptr0: "<<&y<<std::endl;
	std::cout<<"Ptr1: "<<m_cclScene<<std::endl;
	//cclCam.need_flags_update = true;
	//cclCam.update(&**this);
	//*(*this)->dicing_camera = cclCam;
	icycles::camera::set_need_flags_update(cclCam,true);
	icycles::camera::update(cclCam,m_cclScene);
	icycles::scene::set_dicing_camera(*m_cclScene,&cclCam);
}

#define DEPS_LOCATION "F:/projects/pragma"
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/osl/lib/oslcomp.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/osl/lib/oslexec.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/osl/lib/oslnoise.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/osl/lib/oslquery.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMAggressiveInstCombine.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMAnalysis.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMAsmParser.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMBinaryFormat.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMBitReader.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMBitWriter.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMBitstreamReader.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMCFGuard.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMCodeGen.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMCore.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMCoroutines.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMCoverage.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDWARFLinker.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDebugInfoCodeView.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDebugInfoDWARF.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDebugInfoGSYM.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDebugInfoMSF.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDebugInfoPDB.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDemangle.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMDlltoolDriver.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMExecutionEngine.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMExtensions.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMFileCheck.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMFrontendOpenACC.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMFrontendOpenMP.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMFuzzMutate.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMGlobalISel.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMHelloNew.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMIRReader.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMInstCombine.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMInstrumentation.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMInterfaceStub.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMInterpreter.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMJITLink.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMLTO.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMLibDriver.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMLineEditor.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMLinker.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMMC.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMMCA.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMMCDisassembler.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMMCJIT.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMMCParser.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMMIRParser.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMObjCARCOpts.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMObject.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMObjectYAML.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMOption.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMOrcJIT.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMOrcShared.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMOrcTargetProcess.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMPasses.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMProfileData.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMRemarks.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMRuntimeDyld.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMScalarOpts.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMSelectionDAG.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMSupport.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMSymbolize.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMTableGen.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMTarget.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMTextAPI.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMTransformUtils.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMVectorize.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMWindowsManifest.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMX86AsmParser.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMX86CodeGen.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMX86Desc.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMX86Disassembler.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMX86Info.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMXRay.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMipo.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/LLVMAsmPrinter.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangAPINotes.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangARCMigrate.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangAST.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangASTMatchers.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangAnalysis.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangBasic.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangCodeGen.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangCrossTU.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangDependencyScanning.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangDirectoryWatcher.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangDriver.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangDynamicASTMatchers.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangEdit.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangFormat.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangFrontend.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangFrontendTool.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangHandleCXX.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangHandleLLVM.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangIndex.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangIndexSerialization.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangLex.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangParse.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangRewrite.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangRewriteFrontend.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangSema.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangSerialization.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangStaticAnalyzerCheckers.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangStaticAnalyzerCore.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangStaticAnalyzerFrontend.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangTesting.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangTooling.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangToolingASTDiff.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangToolingCore.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangToolingInclusions.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangToolingRefactoring.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangToolingSyntax.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/clangTransformer.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openjpeg/lib/openjp2.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/pugixml/lib/pugixml.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/embree3.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/embree_avx.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/embree_avx2.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/embree_sse42.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/lexers.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/math.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/simd.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/sys.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/embree/lib/tasking.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openvdb/lib/openvdb.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openimageio/lib/OpenImageIO.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openimageio/lib/OpenImageIO_Util.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openimagedenoise/lib/OpenImageDenoise.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openimagedenoise/lib/common.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openimagedenoise/lib/dnnl.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/opensubdiv/lib/osdCPU.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/opensubdiv/lib/osdGPU.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/opencolorio/lib/OpenColorIO.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/opencolorio/lib/libyaml-cpp.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/opencolorio/lib/pystring.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/webp/lib/webp.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/webp/lib/webpdemux.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/webp/lib/webpmux.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/alembic/lib/Alembic.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_util.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_device.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_kernel.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_graph.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_subd.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_bvh.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/extern_cuew.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_integrator.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_kernel_osl.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_scene.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/cycles_session.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/cycles/build/lib/RelWithDebInfo/extern_sky.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/llvm/lib/libclang.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_filesystem-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_regex-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_system-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_thread-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_date_time-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_wave-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_chrono-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_atomic-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/boost/lib/libboost_serialization-vc142-mt-x64-1_78.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openexr/lib/OpenEXR_s.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openexr/lib/OpenEXRCore_s.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/opencolorio/lib/libexpatMD.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/imath/lib/Imath_s.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openexr/lib/Iex_s.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openexr/lib/IlmThread_s.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/openexr/lib/OpenEXRUtil_s.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/jpeg/lib/libjpeg.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/gflags/build_files/lib/RelWithDebInfo/gflags_static.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/tiff/lib/libtiff.lib")
#pragma comment(lib,DEPS_LOCATION "/deps/lib/win64_vc15/tbb/lib/tbb.lib")

#pragma comment(lib,"F:/projects/pragma/build/third_party_libs/libpng/RelWithDebInfo/libpng16.lib")
#pragma comment(lib,"F:/projects/pragma/build/third_party_libs/zlib/RelWithDebInfo/zlib.lib")

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
		cclLight = icycles::light::create(); // Object will be removed automatically by cycles
		m_cclScene->lights.push_back(cclLight);
		m_lightToCclLight[&light] = cclLight;
	}
	icycles::light::set_tfm(*cclLight,ccl::transform_identity());
	switch(light.GetType())
	{
	case unirender::Light::Type::Spot:
		icycles::light::set_light_type(*cclLight,ccl::LightType::LIGHT_SPOT);
		break;
	case unirender::Light::Type::Directional:
		icycles::light::set_light_type(*cclLight,ccl::LightType::LIGHT_DISTANT);
		break;
	case unirender::Light::Type::Area:
		icycles::light::set_light_type(*cclLight,ccl::LightType::LIGHT_AREA);
		break;
	case unirender::Light::Type::Background:
		icycles::light::set_light_type(*cclLight,ccl::LightType::LIGHT_BACKGROUND);
		break;
	case unirender::Light::Type::Triangle:
		icycles::light::set_light_type(*cclLight,ccl::LightType::LIGHT_TRIANGLE);
		break;
	case unirender::Light::Type::Point:
	default:
		icycles::light::set_light_type(*cclLight,ccl::LightType::LIGHT_POINT);
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
		icycles::light::set_dir(*cclLight,ToCyclesNormal(forward));
		icycles::light::set_spot_smooth(*cclLight,light.GetBlendFraction());
		icycles::light::set_spot_angle(*cclLight,umath::deg_to_rad(light.GetOuterConeAngle()));
		break;
	}
	case unirender::Light::Type::Directional:
	{
		auto &rot = light.GetRotation();
		auto forward = uquat::forward(rot);
		icycles::light::set_dir(*cclLight,ToCyclesNormal(forward));
		break;
	}
	case unirender::Light::Type::Area:
	{
		auto &axisU = light.GetAxisU();
		auto &axisV = light.GetAxisV();
		auto sizeU = light.GetSizeU();
		auto sizeV = light.GetSizeV();
		icycles::light::set_axisu(*cclLight,ToCyclesNormal(axisU));
		icycles::light::set_axisv(*cclLight,ToCyclesNormal(axisV));
		icycles::light::set_sizeu(*cclLight,ToCyclesLength(sizeU));
		icycles::light::set_sizev(*cclLight,ToCyclesLength(sizeV));
		icycles::light::set_round(*cclLight,light.IsRound());

		auto &rot = light.GetRotation();
		auto forward = uquat::forward(rot);
		icycles::light::set_dir(*cclLight,ToCyclesNormal(forward));
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

	auto apiData = GetApiData();
	auto customLightIntensityMultiplier = 1.f;
	if(apiData.GetFromPath("lightIntensityMultiplier")(customLightIntensityMultiplier))
		watt = customLightIntensityMultiplier;
	else
	{
		static auto lightIntensityMultiplier = 200.f;
		watt *= scene.GetLightIntensityFactor() *lightIntensityMultiplier;
	}
	auto &color = light.GetColor();
	icycles::light::set_strength(*cclLight,ccl::float3{color.r,color.g,color.b} *watt);
	icycles::light::set_size(*cclLight,ToCyclesLength(light.GetSize()));
	icycles::light::set_co(*cclLight,ToCyclesPosition(light.GetPos()));

	icycles::light::set_max_bounces(*cclLight,1'024);
	icycles::light::set_map_resolution(*cclLight,2'048);

	auto uuid = util::uuid_to_string(light.GetUuid());
	auto udmLight = apiData.GetFromPath("cycles/scene/actors/" +uuid);
	if(udmLight)
	{
		uint32_t maxBounces;
		if(udmLight["maxBounces"](maxBounces))
			icycles::light::set_max_bounces(*cclLight,maxBounces);

		uint32_t mapResolution;
		if(udmLight["mapResolution"](mapResolution))
			icycles::light::set_map_resolution(*cclLight,mapResolution);
	}

	icycles::light::tag_update(*cclLight,m_cclScene);
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
	nodeEmission.SetProperty(unirender::nodes::emission::IN_STRENGTH,1.f);
	nodeEmission.SetProperty(unirender::nodes::emission::IN_COLOR,Vector3{1.f,1.f,1.f});
	desc->Link(nodeEmission.GetOutputSocket("emission"),outputNode.GetInputSocket("surface"));

	auto shader = CCLShader::Create(*this,*desc);
	icycles::light::set_shader(*cclLight,**shader);
	m_lightToShader[&light] = shader;
}

icycles::BufferParams unirender::cycles::Renderer::GetBufferParameters() const
{
	auto &cam = m_scene->GetCamera();
	icycles::BufferParams bufferParams;
	icycles::util::create_buffer_params(bufferParams);
	icycles::buffer_params::set_width(*bufferParams,cam.GetWidth());
	icycles::buffer_params::set_height(*bufferParams,cam.GetHeight());
	icycles::buffer_params::set_full_width(*bufferParams,cam.GetWidth());
	icycles::buffer_params::set_full_height(*bufferParams,cam.GetHeight());
	SetupRenderSettings(*m_cclScene,*m_cclSession,*bufferParams,m_renderMode,m_scene->GetSceneInfo().maxTransparencyBounces);
	return bufferParams;
}

float unirender::cycles::Renderer::GetProgress() const
{
	return m_cclSession->progress.get_progress();
}
bool unirender::cycles::Renderer::Stop() {return false;}
bool unirender::cycles::Renderer::Pause()
{
	if(!m_cclSession)
		return false;
	icycles::session::set_pause(*m_cclSession,true);
	return true;
}
bool unirender::cycles::Renderer::Resume()
{
	if(!m_cclSession)
		return false;
	icycles::session::set_pause(*m_cclSession,false);
	return true;
}
bool unirender::cycles::Renderer::Suspend() {return false;}
bool unirender::cycles::Renderer::Export(const std::string &path) {return false;}
void unirender::cycles::Renderer::Wait()
{
	if(m_cclSession)
		icycles::session::wait(*m_cclSession);
}

void unirender::cycles::Renderer::ApplyPostProcessing(uimg::ImageBuffer &imgBuffer,unirender::Scene::RenderMode renderMode)
{
	// For some reason the image is flipped horizontally when rendering an image,
	// so we'll just flip it the right way here
	auto flipHorizontally = unirender::Scene::IsRenderSceneMode(renderMode);
	if(icycles::camera::get_camera_type(*m_cclScene->camera) == ccl::CameraType::CAMERA_PANORAMA)
	{
		switch(icycles::camera::get_panorama_type(*m_cclScene->camera))
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
	if(Scene::IsLightmapRenderMode(renderMode))
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
  icycles::SceneParams scene_params;
  icycles::SessionParams session_params;
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

static icycles::BufferParams &session_buffer_params(Options &opts)
{
	icycles::BufferParams buffer_params;
	icycles::util::create_buffer_params(buffer_params);
	icycles::buffer_params::set_width(*buffer_params,opts.width);
	icycles::buffer_params::set_height(*buffer_params,opts.height);
	icycles::buffer_params::set_full_width(*buffer_params,opts.width);
	icycles::buffer_params::set_full_height(*buffer_params,opts.height);
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
  /* get status */
  auto &oProgress = icycles::session::get_progress(*opts.session);
  double progress = icycles::progress::get_progress(oProgress);
  icycles::CString cstatus {};
  icycles::CString csubstatus {};
  icycles::progress::get_progress_status(oProgress,&cstatus.cstr,cstatus.len,&csubstatus.cstr,csubstatus.len);

  auto status = static_cast<std::string>(cstatus);
  auto substatus = static_cast<std::string>(csubstatus);
  if (substatus != "")
    status += ": " + substatus;

  /* print status */
  //status = ccl::string_printf("Progress %05.2f   %s", (double)progress * 100, status.c_str());
  //session_print(status);
}

#include <app/oiio_output_driver.h>

class COIIOOutputDriver : public icycles::OutputDriver {
 public:
  typedef ccl::function<void(const ccl::string &)> LogFunction;

  COIIOOutputDriver(const ccl::string_view filepath, const ccl::string_view pass, LogFunction log);
  virtual ~COIIOOutputDriver();

  void write_render_tile(const Tile &tile) override;

 protected:
  std::string filepath_;
  std::string pass_;
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
  if (!(icycles::output_driver_tile::get_size(const_cast<ccl::OutputDriver::Tile&>(tile)) == icycles::output_driver_tile::get_full_size(const_cast<ccl::OutputDriver::Tile&>(tile)))) {
    return;
  }

  //log_(ccl::string_printf("Writing image %s", filepath_.c_str()));

  std::shared_ptr<ccl::ImageOutput> image_output;
  icycles::image_output::create(filepath_.c_str(),image_output);
  if (image_output == nullptr) {
    //log_("Failed to create image file");
    return;
  }

  const int width = icycles::output_driver_tile::get_size(const_cast<ccl::OutputDriver::Tile&>(tile)).x;
  const int height = icycles::output_driver_tile::get_size(const_cast<ccl::OutputDriver::Tile&>(tile)).y;

  std::shared_ptr<ccl::ImageSpec> spec;
  icycles::util::create_image_spec(width, height, 4, ccl::TypeDesc::FLOAT,spec);
  if (!icycles::image_output::open(*image_output,filepath_.c_str(), *spec)) {
    //log_("Failed to create image file");
    return;
  }

  ccl::vector<float> pixels(width * height * 4);
  if (!icycles::output_driver_tile::get_pass_pixels(tile,pass_.c_str(), 4, pixels.data())) {
    //log_("Failed to read render pass pixels");
    return;
  }

  /* Manipulate offset and stride to convert from bottom-up to top-down convention. */
  icycles::image_output::write_image(*image_output,ccl::TypeDesc::FLOAT,
                            pixels.data() + (height - 1) * width * 4,
                            ccl::AutoStride,
                            -width * 4 * sizeof(float),
                            ccl::AutoStride);
  icycles::image_output::close(*image_output);
}

void unirender::cycles::Renderer::AddDebugSky()
{
	auto *shader = m_cclScene->default_background;
	auto *graph = icycles::shader_graph::create();

	const ccl::NodeType *skyTexNodeType = icycles::util::find_node_type("sky_texture");
	auto skyTex = (ccl::SkyTextureNode*)icycles::node_type::create_node(*skyTexNodeType);
	icycles::shader_node::set_owner(*skyTex,graph);
	icycles::sky_texture_node::set_sky_type(*skyTex,ccl::NodeSkyType::NODE_SKY_HOSEK);
	icycles::node::set_name(*skyTex,"tex");
	icycles::shader_graph::add(*graph,skyTex);

	const ccl::NodeType *bgShaderNodeType = icycles::util::find_node_type("background_shader");
	auto bgShader = (ccl::BackgroundNode *)icycles::node_type::create_node(*bgShaderNodeType);
	icycles::shader_node::set_owner(*bgShader,graph);
	icycles::background_node::set_strength(*bgShader,8.f);
	icycles::background_node::set_color(*bgShader,{1.f,0.f,0.f});
	icycles::node::set_name(*bgShader,"bg");
	icycles::shader_graph::add(*graph,bgShader);

	icycles::shader_graph::connect(*graph,find_output_socket(*skyTex,"color"),find_input_socket(*bgShader,"color"));
	icycles::shader_graph::connect(*graph,find_output_socket(*bgShader,"background"),find_input_socket(*icycles::shader_graph::output(*graph),"surface"));

	icycles::shader::set_graph(*shader,graph);
	icycles::shader::tag_update(*shader,m_cclScene);
}

ccl::Mesh *unirender::cycles::Renderer::AddDebugMesh()
{
	auto *cclMesh = icycles::mesh::create();
	icycles::scene::add_geometry(*m_cclScene,cclMesh);

	icycles::node::set_name(*cclMesh,"floor");
	auto *mesh = cclMesh;
	ccl::array<ccl::float3> P_array {};
	P_array.push_back_slow(ccl::float3{-3.f,3.f,0.f});
	P_array.push_back_slow(ccl::float3{3.f,3.f,0.f});
	P_array.push_back_slow(ccl::float3{3.f,-3.f,0.f});
	P_array.push_back_slow(ccl::float3{-3.f,-3.f,0.f});
	icycles::mesh::set_verts(*mesh,P_array);

	size_t num_triangles = 0;
	ccl::vector<int> nverts {};
	nverts.push_back(4);
	for (size_t i = 0; i < nverts.size(); i++)
		num_triangles += nverts[i] - 2;
	icycles::mesh::reserve_mesh(*mesh,icycles::mesh::get_verts(*mesh).size(), num_triangles);

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

			icycles::mesh::add_triangle(*mesh,v0, v1, v2, ishader, smooth);
		}

		index_offset += nverts[i];
	}

	if (icycles::geometry::need_attribute(*mesh, m_cclScene, ccl::ATTR_STD_GENERATED)) {
		ccl::Attribute *attr = icycles::attribute_set::add(icycles::geometry::get_attributes(*mesh),ccl::ATTR_STD_GENERATED);
		memcpy(
			icycles::attribute::data_float3(*attr), icycles::mesh::get_verts(*mesh).data(), sizeof(ccl::float3) * icycles::mesh::get_verts(*mesh).size()
		);
	}
	return cclMesh;
}
ccl::Object *unirender::cycles::Renderer::AddDebugObject()
{
	auto *mesh = AddDebugMesh();
	ccl::Object *object = icycles::object::create();
	icycles::object::set_geometry(*object,mesh);
	ccl::Transform t = ccl::transform_identity();
	icycles::object::set_tfm(*object,t);
	m_cclScene->objects.push_back(object);
	return object;
}
void unirender::cycles::Renderer::AddDebugLight()
{
	auto *shader = icycles::shader::create();
	icycles::node::set_name(*shader,"point_shader");
	ccl::ShaderGraph *graph = icycles::shader_graph::create();
	
	const ccl::NodeType *emissionType = icycles::util::find_node_type("emission");
	auto emissionNode = (ccl::EmissionNode *)icycles::node_type::create_node(*emissionType);
	icycles::shader_node::set_owner(*emissionNode,graph);
	icycles::node::set_name(*emissionNode,"emission");
	icycles::emission_node::set_color(*emissionNode,ccl::float3{0.8f,0.1f,0.1f} *100.f);
	icycles::shader_graph::add(*graph,emissionNode);

	icycles::shader_graph::connect(*graph,find_output_socket(*emissionNode,"emission"),find_input_socket(*icycles::shader_graph::output(*graph),"surface"));
	
	icycles::shader::set_graph(*shader,graph);
	icycles::shader::tag_update(*shader,m_cclScene);
	m_cclScene->shaders.push_back(shader);

	//

	auto *light = icycles::light::create();
	m_cclScene->lights.push_back(light);
	icycles::light::set_light_type(*light,ccl::LightType::LIGHT_POINT);
	icycles::light::set_shader(*light,shader);
	icycles::light::set_size(*light,1.f);
	icycles::light::set_co(*light,{0.f,0.f,1.f});
}
ccl::Shader *unirender::cycles::Renderer::AddDebugShader()
{
	auto *shader = icycles::shader::create();
	icycles::node::set_name(*shader,"shader_test");
	ccl::ShaderGraph *graph = icycles::shader_graph::create();
	
	const ccl::NodeType *nodeTypeGlossy = icycles::util::find_node_type("glossy_bsdf");
	auto glossyNode = (ccl::GlossyBsdfNode *)icycles::node_type::create_node(*nodeTypeGlossy);
	icycles::shader_node::set_owner(*glossyNode,graph);
	icycles::node::set_name(*glossyNode,"floor_closure2");
	icycles::node::set_float(*glossyNode,*find_type_input(*glossyNode,"roughness"),0.2f);
	icycles::node::set_cstring(*glossyNode,*find_type_input(*glossyNode,"distribution"),"beckmann");
	icycles::shader_graph::add(*graph,glossyNode);

	const ccl::NodeType *nodeTypeCheckerTex = icycles::util::find_node_type("checker_texture");
	auto checkerNode = (ccl::CheckerTextureNode *)icycles::node_type::create_node(*nodeTypeCheckerTex);
	icycles::shader_node::set_owner(*checkerNode,graph);
	icycles::node::set_name(*checkerNode,"checker2");
	icycles::node::set_float3(*checkerNode,*find_type_input(*checkerNode,"color1"),ccl::float3{0.8f,0.8f,0.8f});
	icycles::node::set_float3(*checkerNode,*find_type_input(*checkerNode,"color2"),ccl::float3{1.f,0.1f,0.1f});
	icycles::shader_graph::add(*graph,checkerNode);
		
	icycles::shader_graph::connect(*graph,find_output_socket(*checkerNode,"color"),find_input_socket(*glossyNode,"color"));
	icycles::shader_graph::connect(*graph,find_output_socket(*glossyNode,"bsdf"),find_input_socket(*icycles::shader_graph::output(*graph),"surface"));
	
	icycles::shader::set_graph(*shader,graph);
	icycles::shader::tag_update(*shader,m_cclScene);
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
	icycles::util::create_scene_params(opts.scene_params);
	icycles::util::create_session_params(opts.session_params);
	icycles::session_params::set_use_auto_tile(*opts.session_params,false);
	icycles::session_params::set_tile_size(*opts.session_params,16);
	icycles::session_params::set_samples(*opts.session_params,20);

	opts.output_pass = "combined";
	opts.session = m_cclSession.get();
	opts.scene = icycles::session::get_scene(*opts.session);

	auto &cam = GetScene().GetCamera();
	SyncCamera(cam);

	PopulateDebugScene();
	for(auto &filepath : xmlFileNames)
		ccl::xml_read_file(opts.scene, filepath.c_str());
	
	if (!opts.output_filepath.empty()) {
		auto driver = std::make_shared<COIIOOutputDriver>(opts.output_filepath, opts.output_pass, session_print);
		icycles::session::set_output_driver(*opts.session,driver->CreateDriver()); // Cycles now has primary ownership of output driver object
	}
	
	auto &progress = icycles::session::get_progress(*opts.session);
	progress.set_update_callback([this]() {std::cout<<"Progress: "<<m_cclSession->progress.get_progress()<<","<<m_cclSession->progress.get_cancel_message()<<","<<m_cclSession->progress.get_error_message()<<std::endl;});
	  
	/* add pass for output. */
	ccl::Pass *pass = icycles::scene::create_node_pass(*opts.scene);
	icycles::node::set_name(*pass,opts.output_pass.c_str());
	icycles::pass::set_type(*pass,ccl::PASS_COMBINED);

	auto useOptix = false;
	if(useOptix)
	{
		using namespace ccl;
		icycles::session_params::set_device_by_type(*opts.session_params,ccl::DeviceType::DEVICE_OPTIX);
	}
	icycles::session_params::set_use_auto_tile(*opts.session_params,false);
	icycles::session_params::set_tile_size(*opts.session_params,0);
	icycles::session_params::set_background(*opts.session_params,true);

	icycles::session::reset(*opts.session,*opts.session_params, *session_buffer_params(opts));
	icycles::session::start(*opts.session);
	
	icycles::session::wait(*opts.session);

	opts.session = nullptr;
	m_cclSession = nullptr;
}

void unirender::cycles::Renderer::PopulateDebugScene()
{
	AddDebugSky();
	auto *obj = AddDebugObject();
	auto *shader = AddDebugShader();
	auto *mesh = icycles::object::get_geometry(*obj);
	AddDebugLight();

	ccl::array<ccl::Node *> used_shaders = icycles::geometry::get_used_shaders(*mesh);
	used_shaders.push_back_slow(shader);
	icycles::geometry::set_used_shaders(*mesh,used_shaders);
}

bool unirender::cycles::Renderer::BeginSceneEdit() {return true;}
bool unirender::cycles::Renderer::EndSceneEdit() {return true;}
bool unirender::cycles::Renderer::AddLiveActor(unirender::WorldObject &actor)
{
	auto *pLight = dynamic_cast<unirender::Light*>(&actor);
	if(!pLight)
		return false;
	GetScene().AddLight(*pLight);
	SyncLight(*m_scene,*pLight);
	auto it = m_lightToShader.find(pLight);
	if(it == m_lightToShader.end())
		return false;
	AddActorToActorMap(actor);
	it->second->Finalize(*m_scene);
	return true;
}
bool unirender::cycles::Renderer::SyncEditedActor(const util::Uuid &uuid)
{
	auto *actor = FindActor(uuid);
	if(!actor)
		return false;
	if(typeid(*actor) == typeid(Camera))
		SyncCamera(static_cast<Camera&>(*actor),true);
	else if(typeid(*actor) == typeid(Light))
		SyncLight(*m_scene,static_cast<Light&>(*actor),true);
	else if(typeid(*actor) == typeid(Object))
	{
		auto &o = *static_cast<Object*>(actor);
		auto it = m_uuidToObject.find(util::uuid_to_string(o.GetUuid()));
		if(it != m_uuidToObject.end())
		{
			auto it2 = m_objectToCclObject.find(it->second);
			if(it2 != m_objectToCclObject.end())
			{
				auto &cclObjInfo = it2->second;
				auto *cclObj = cclObjInfo.object;
				auto *geo = icycles::object::get_geometry(*cclObj);
				if(geo && icycles::geometry::get_transform_applied(*geo))
				{
					// Transforms have already been applied; Restore original vertices
					// and apply new transforms
					auto *cclMesh = static_cast<ccl::Mesh*>(geo);
					auto *mesh = m_cclMeshToMesh.find(cclMesh)->second;
					auto &verts = mesh->GetVertices();
					auto &cclVerts = icycles::mesh::get_verts(*cclMesh);
					for(auto i=decltype(verts.size()){0u};i<verts.size();++i)
						cclVerts[i] = ToCyclesPosition(verts[i]);

					auto lastPose = cclObjInfo.lastUpdatePose;
					cclObjInfo.lastUpdatePose = o.GetPose();
					auto cclPose = ToCyclesTransform(o.GetPose());
					icycles::geometry::apply_transform(*geo,cclPose,false);
					icycles::geometry::set_transform_normal(*geo,icycles::transform::transposed_inverse(cclPose));

					icycles::object::set_tfm(*cclObj,ToCyclesTransform(o.GetPose(),true));
					icycles::object::tag_tfm_modified(*cclObj);
					icycles::object::tag_update(*cclObj,m_cclScene);
				}
				else
				{
					icycles::object::set_tfm(*cclObj,ToCyclesTransform(o.GetPose(),true));
					icycles::object::tag_tfm_modified(*cclObj);
					icycles::object::tag_update(*cclObj,m_cclScene);
				}
			}
		}
	}
	else
		return false;
	umath::set_flag(m_stateFlags,StateFlags::ReloadSessionScheduled);
	return true;
}

bool unirender::cycles::Renderer::Initialize(unirender::Scene &scene,std::string &outErr)
{
	auto devInfo = InitializeDevice(scene,outErr);
	if(!devInfo)
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
	
		auto availableDenoisers = icycles::device_info::get_denoisers(*devInfo);
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
		AddOutput(OUTPUT_AO);
		break;
	case unirender::Scene::RenderMode::BakeNormals:
		AddOutput(OUTPUT_COLOR);
		break;
	case unirender::Scene::RenderMode::BakeDiffuseLighting:
		AddOutput(OUTPUT_DIFFUSE);
		break;
	case unirender::Scene::RenderMode::BakeDiffuseLightingSeparate:
		AddOutput(OUTPUT_DIFFUSE_DIRECT);
		AddOutput(OUTPUT_DIFFUSE_INDIRECT);
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

	auto *integrator = icycles::scene::get_integrator(*m_cclScene);
	if(denoiserType == ccl::DenoiserType::DENOISER_NONE)
		icycles::integrator::set_use_denoise(*integrator,false);
	else
	{
		icycles::integrator::set_use_denoise(*integrator,true);
		icycles::integrator::set_denoiser_type(*integrator,denoiserType);
		icycles::integrator::set_denoise_start_sample(*integrator,1);
		// m_cclScene->integrator->set_denoiser_prefilter(ccl::DenoiserPrefilter::DENOISER_PREFILTER_FAST);
		if(umath::is_flag_set(m_flags,Flags::EnableLiveEditing))
		{
			icycles::integrator::set_use_denoise_pass_albedo(*integrator,true);
			icycles::integrator::set_use_denoise_pass_normal(*integrator,false);
		}
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

	icycles::util::create_session_params(m_sessionParams);
	icycles::session_params::copy(icycles::session::get_params(*m_cclSession),*m_sessionParams);

	icycles::util::create_buffer_params(m_bufferParams);
	icycles::buffer_params::copy(*bufferParams,*m_bufferParams);

	icycles::session::reset(*m_cclSession,*m_sessionParams,*m_bufferParams);

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
	if(/*createInfo.progressive && */GetTileSize() > 0)
	{
		auto *camera = icycles::scene::get_camera(*m_cclScene);
		auto w = icycles::camera::get_full_width(*camera);
		auto h = icycles::camera::get_full_height(*camera);
		m_tileManager.Initialize(w,h,GetTileSize(),GetTileSize(),m_deviceType == Scene::DeviceType::CPU,createInfo.exposure,m_scene->GetGamma(),m_colorTransformProcessor.get());
		bool flipHorizontally = true;
		if(icycles::camera::get_camera_type(*camera) == ccl::CameraType::CAMERA_PANORAMA)
		{
			switch(icycles::camera::get_panorama_type(*camera))
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

	if(m_scene->HasBakeTarget() && !InitializeBakingData())
	{
		outErr = "Failed to initialize bake data.";
		return false;
	}

	Vector2i tileSize {cam.GetWidth(),cam.GetHeight()};
	if(m_cclSession->params.use_auto_tile)
		tileSize = {m_cclSession->params.tile_size,m_cclSession->params.tile_size};
	auto outputDriver = std::make_shared<OutputDriver>(passes,cam.GetWidth(),cam.GetHeight());
	if(m_bakeData)
		static_cast<OutputDriver&>(*outputDriver).SetBakeData(*m_bakeData);
	m_outputDriver = outputDriver.get();
	if(IsDisplayDriverEnabled() && !m_bakeData)
	{
		auto displayDriver = std::make_unique<DisplayDriver>(m_tileManager,cam.GetWidth(),cam.GetHeight());
		displayDriver->UpdateTileResolution(tileSize.x,tileSize.y);
		m_displayDriver = displayDriver.get();
		icycles::session::set_display_driver(*m_cclSession,displayDriver.release());
	}
	icycles::session::set_output_driver(*m_cclSession,outputDriver->CreateDriver()); // Cycles now has primary ownership of output driver object

	//
	
	auto *background = icycles::scene::get_background(*m_cclScene);
	if(scene.GetRenderMode() == Scene::RenderMode::BakeAmbientOcclusion)
	{
		icycles::integrator::set_use_direct_light(*integrator,false);
		icycles::integrator::set_use_indirect_light(*integrator,false);
		icycles::background::set_transparent(*background,true);
	}
	if(
		Scene::IsLightmapRenderMode(scene.GetRenderMode()) ||
		scene.GetRenderMode() == Scene::RenderMode::BakeNormals
	)
		icycles::background::set_transparent(*background,true);
	if(ShouldUseTransparentSky())
		icycles::background::set_transparent(*background,true);

	auto *bakeTarget = m_scene->GetBakeTargetName();
	if(bakeTarget)
		icycles::bake_manager::set(*icycles::scene::get_bake_manager(*m_cclScene),m_cclScene,bakeTarget->c_str());

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
				icycles::geometry::get_used_shaders(*cclMesh)[i] = **cclShader;
			}
			icycles::mesh::tag_update(*cclMesh,m_cclScene,false);
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
	icycles::session::reset(*m_cclSession,icycles::session::get_params(*m_cclSession),*bufferParams); // We only need the normals and albedo colors for the first sample

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
	auto &sessionParams = icycles::session::get_params(*m_cclSession);
	icycles::session_params::set_samples(sessionParams,sampleCount);
	icycles::session::reset(*m_cclSession,sessionParams,*bufferParams); // We only need the normals and albedo colors for the first sample

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
	icycles::scene::set_default_background(*m_cclScene,**cclShader);
	icycles::shader::tag_update(***cclShader,m_cclScene);

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
	icycles::session::wait(*m_cclSession);
	icycles::progress::reset(icycles::session::get_progress(*m_cclSession));
}
void unirender::cycles::Renderer::Restart()
{
	auto &createInfo = m_scene->GetCreateInfo();
	if(createInfo.progressive)
		m_tileManager.Reload(false);

	icycles::session::start(*m_cclSession);
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
	auto *light = icycles::light::create(); // Object will be removed automatically by cycles
	icycles::light::set_tfm(*light,ccl::transform_identity());

	m_cclScene->lights.push_back(light);
	icycles::light::set_light_type(*light,ccl::LightType::LIGHT_BACKGROUND);
	icycles::light::set_map_resolution(*light,2'048);
	icycles::light::set_shader(*light,m_cclScene->default_background);
	icycles::light::set_use_mis(*light,true);
	icycles::light::set_max_bounces(*light,1'024);
	icycles::light::tag_update(*light,m_cclScene);
}

util::ParallelJob<uimg::ImageLayerSet> unirender::cycles::Renderer::StartRender()
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
#ifdef __linux__
#define DLLEXPORT __attribute__((visibility("default")))
#else
#define DLLEXPORT __declspec(dllexport)
#endif
extern "C" {
	bool DLLEXPORT create_renderer(const unirender::Scene &scene,unirender::Renderer::Flags flags,std::shared_ptr<unirender::Renderer> &outRenderer,std::string &outErr)
	{
		unirender::Scene::SetKernelPath(util::get_program_path() +"/modules/unirender/cycles");
		outRenderer = unirender::cycles::Renderer::Create(scene,outErr,flags);
		return outRenderer != nullptr;
	}
};
