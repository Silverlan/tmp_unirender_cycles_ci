/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2022 Silverlan
*/

#ifndef __ICYCLES_HPP__
#define __ICYCLES_HPP__

#ifdef _WIN32
#define ENABLE_CYCLES_LOGGING
#endif

#include <scene/shader.h>
#include <scene/shader_nodes.h>
#include <scene/camera.h>
#include <scene/scene.h>
#include <scene/mesh.h>
#include <scene/light.h>
#include <scene/object.h>
#include <scene/hair.h>
#include <scene/background.h>
#include <scene/bake.h>
#include <scene/integrator.h>
#include <session/session.h>
#include <session/output_driver.h>
#include <session/display_driver.h>
#include <util/path.h>
#include <cinttypes>
#include <cstring>
#include <memory>

#ifdef ENABLE_CYCLES_LOGGING
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#endif

#ifdef ICYCLES_DEFINE

#ifdef __linux__
#define DLLICYCLES __attribute__((visibility("default")))
#else
#define DLLICYCLES __declspec(dllexport)
#endif

#endif

#define COMMA ,

#ifdef ICYCLES_DEFINE

	#define DEFINE_SIMPLE_FUNCTION(ns,funcName,returnType,parameters,arguments,impl) \
		extern "C" { \
		DLLICYCLES returnType icycles##_##ns##_##funcName parameters \
		{ \
			impl; \
		} \
		}

	#define DEFINE_SIMPLE_METHOD(ns,funcName,returnType,parameters,object,...) \
			extern "C" { \
			DLLICYCLES returnType icycles##_##ns##_##funcName parameters \
			{ \
				return object.funcName(__VA_ARGS__); \
			} \
			}

	#define DEFINE_SIMPLE_MEMBER(ns,funcName,memberName,returnType,parameters,object) \
			extern "C" { \
			DLLICYCLES returnType icycles##_##ns##_##funcName parameters \
			{ \
				return object.memberName; \
			} \
			}

	#define DEFINE_SIMPLE_MEMBER_GETTER(ns,memberName,returnType,parameters,object) \
			extern "C" { \
			DLLICYCLES returnType icycles##_##ns##_##get_##memberName parameters \
			{ \
				return object.memberName; \
			} \
			}

	#define DEFINE_SIMPLE_MEMBER_SETTER(ns,memberName,parameters,object) \
			extern "C" { \
			DLLICYCLES void icycles##_##ns##_##set_##memberName parameters \
			{ \
				object.memberName = value; \
			} \
			}

	#define DEFINE_SIMPLE_METHOD_0(ns,funcName,returnType,parameters,object) \
		DEFINE_SIMPLE_METHOD(ns,funcName,returnType,parameters,object,)

	#define DEFINE_SIMPLE_METHOD_N(ns,funcName,returnType,parameters,object,...) \
		DEFINE_SIMPLE_METHOD(ns,funcName,returnType,parameters,object,__VA_ARGS__)

	namespace icycles::detail
	{
		DLLICYCLES void str_to_cstr(const std::string &str,char **outStr, size_t& outStrLen)
		{
			outStrLen = str.length();
			*outStr = new char[outStrLen + 1];
			memcpy(*outStr, str.data(), outStrLen + 1);
		}
		DLLICYCLES void str_to_cstr(const char *str,char **outStr, size_t& outStrLen)
		{
			outStrLen = strlen(str);
			*outStr = new char[outStrLen + 1];
			memcpy(*outStr, str, outStrLen + 1);
		}

		template<typename T>
			DLLICYCLES void free(T *v) {delete v;}
	};

#else

	#ifdef ICYCLES_DEFINE_LOOKUP_FUNCTIONS
		#define DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,funcName,returnType,parameters,...) \
			namespace icycles \
			{ \
				namespace ns \
				{ \
					returnType funcName parameters \
					{ \
						static auto *addr = icycles::find_symbol<decltype(&funcName)>("icycles_" #ns "_" #funcName); \
						return addr(__VA_ARGS__); \
					} \
				} \
			}
	#else
		#define DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,funcName,returnType,parameters,...) \
			namespace icycles \
			{ \
				namespace ns \
				{ \
					returnType funcName parameters; \
				} \
			}
	#endif

	#define DEFINE_SIMPLE_FUNCTION(ns,funcName,returnType,parameters,arguments,impl) \
		DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,funcName,returnType,parameters,arguments)

	#define DEFINE_SIMPLE_METHOD_WITH_OBJECT(ns,funcName,returnType,parameters,object,arguments) \
		DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,funcName,returnType,parameters,object COMMA arguments)

	#define DEFINE_SIMPLE_METHOD_0(ns,funcName,returnType,parameters,object) \
		DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,funcName,returnType,parameters,object)

	#define DEFINE_SIMPLE_METHOD_N(ns,funcName,returnType,parameters,object,...) \
		DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,funcName,returnType,parameters,object,__VA_ARGS__)

	#define DEFINE_SIMPLE_MEMBER(ns,funcName,memberName,returnType,parameters,object) \
			DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,funcName,returnType,parameters,object)

	#define DEFINE_SIMPLE_MEMBER_GETTER(ns,memberName,returnType,parameters,object) \
			DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,get_##memberName,returnType,parameters,object)

	#define DEFINE_SIMPLE_MEMBER_SETTER(ns,memberName,parameters,object) \
			DEFINE_SIMPLE_METHOD_NO_OBJECT(ns,set_##memberName,void,parameters,object COMMA value)

#endif

#define DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(ns,memberName,returnType,parametersGetter,parametersSetter,object) \
		DEFINE_SIMPLE_MEMBER_GETTER(ns,memberName,returnType,parametersGetter,object) \
		DEFINE_SIMPLE_MEMBER_SETTER(ns,memberName,parametersSetter,object)

namespace icycles
{
	using Session = std::shared_ptr<ccl::Session>;
	using SessionParams = std::shared_ptr<ccl::SessionParams>;
	using SceneParams = std::shared_ptr<ccl::SceneParams>;
	using BufferParams = std::shared_ptr<ccl::BufferParams>;

	namespace impl
	{
		class BaseOutputDriver
			: public ccl::OutputDriver
		{
		public:
			virtual void write_render_tile(const Tile &tile) override
			{
				if(m_writeRenderTile)
					m_writeRenderTile(m_userData.get(),tile);
			}

			virtual bool update_render_tile(const Tile & tile) override
			{
				if(m_updateRenderTile)
					return m_updateRenderTile(m_userData.get(),tile);
				return false;
			}

			virtual bool read_render_tile(const Tile & tile) override
			{
				if(m_readRenderTile)
					return m_readRenderTile(m_userData.get(),tile);
				return false;
			}

			void Setup(
				std::shared_ptr<void> userData,
				void(*writeRenderTile)(void*,const Tile&),
				bool(*updateRenderTile)(void*,const Tile&),
				bool(*readRenderTile)(void*,const Tile&)
			)
			{
				m_userData = userData;
				m_writeRenderTile = writeRenderTile;
				m_updateRenderTile = updateRenderTile;
				m_readRenderTile = readRenderTile;
			}
		private:
			std::shared_ptr<void> m_userData = nullptr;
			void(*m_writeRenderTile)(void*,const Tile&) = nullptr;
			bool(*m_updateRenderTile)(void*,const Tile&) = nullptr;
			bool(*m_readRenderTile)(void*,const Tile&) = nullptr;
		};
	};

	enum class ShaderNodeType : uint32_t
	{
		None = 0,
		Math,
		Mapping,
		Mix,
		VectorMath,
		VectorTransform,
		ImageTexture,
		NormalMap,
		Other
	};
};

// attribute
DEFINE_SIMPLE_FUNCTION(
	attribute,set_name, /* namespace, function name */
	void, /* returnType */
	(ccl::Attribute& attribute, const char* name), /* Parameters */
	attribute COMMA name, /* Arguments */
	{
		attribute.name = name;
	}
)
DEFINE_SIMPLE_FUNCTION(
	attribute,get_name, /* namespace, function name */
	void, /* returnType */
	(const ccl::Attribute& attribute, char **outStr, std::size_t& outStrLen), /* Parameters */
	attribute COMMA outStr COMMA outStrLen, /* Arguments */
	{
		return icycles::detail::str_to_cstr(attribute.name.c_str(),outStr,outStrLen);
	}
)

DEFINE_SIMPLE_METHOD_0(attribute, data_float3, ccl::float3*,
	(ccl::Attribute& attribute), attribute )
DEFINE_SIMPLE_METHOD_0(attribute, data_float2, ccl::float2*,
	(ccl::Attribute& attribute), attribute )
DEFINE_SIMPLE_METHOD_0(attribute, data_float, float*,
	(ccl::Attribute& attribute), attribute )
DEFINE_SIMPLE_METHOD_0(attribute, data_sizeof, size_t,
	(ccl::Attribute& attribute), attribute )

DEFINE_SIMPLE_METHOD_N(attribute, resize, /* namespace, function name */
	void, /* returnType */
	(ccl::Attribute &attribute, std::size_t size), /* Parameters */
	attribute, /* Object */
	size /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(attribute, data, /* namespace, function name */
	char*, /* returnType */
	(ccl::Attribute &attribute), /* Parameters */
	attribute /* Object */
)

// attribute_set
DEFINE_SIMPLE_FUNCTION(
	attribute_set,find_by_layer, /* namespace, function name */
	ccl::Attribute*, /* returnType */
	(const ccl::AttributeSet& attributeSet, const char* layerName), /* Parameters */
	attributeSet COMMA layerName, /* Arguments */
	{
		return attributeSet.find(ccl::ustring{ layerName });
	}
)
DEFINE_SIMPLE_METHOD_N(attribute_set, add, ccl::Attribute*,
	(ccl::AttributeSet& attributeSet, ccl::AttributeStandard strd), attributeSet,
	strd)
DEFINE_SIMPLE_METHOD_N(attribute_set, find, ccl::Attribute*,
	(const ccl::AttributeSet& attributeSet, ccl::AttributeStandard strd),
	attributeSet, strd)
DEFINE_SIMPLE_FUNCTION(
	attribute_set,add_by_type, /* namespace, function name */
	ccl::Attribute*, /* returnType */
	(ccl::AttributeSet& attributeSet, const char *name, const char *type, ccl::AttributeElement element), /* Parameters */
	attributeSet COMMA name COMMA type COMMA element, /* Arguments */
	{
		if(type == "vector")
			return attributeSet.add(ccl::ustring{name},ccl::TypeDesc::TypeVector,element);
		else if(type == "float")
			return attributeSet.add(ccl::ustring{name},ccl::TypeDesc::TypeFloat,element);
		throw std::invalid_argument{"Invalid type '" +std::string{type} +"'!"};
	}
)

// shader_graph
DEFINE_SIMPLE_FUNCTION(
	shader_graph,create, /* namespace, function name */
	ccl::ShaderGraph*, /* returnType */
	(), /* Parameters */
	, /* Arguments */
	{
		return new ccl::ShaderGraph{};
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_graph,destroy, /* namespace, function name */
	void, /* returnType */
	(ccl::ShaderGraph *shaderGraph), /* Parameters */
	shaderGraph, /* Arguments */
	{
		icycles::detail::free(shaderGraph);
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_graph,get_node_count, /* namespace, function name */
	std::size_t, /* returnType */
	(const ccl::ShaderGraph &shaderGraph), /* Parameters */
	shaderGraph, /* Arguments */
	{
		return shaderGraph.nodes.size();
	}
)
DEFINE_SIMPLE_METHOD_N(shader_graph, simplify, /* namespace, function name */
	void, /* returnType */
	(ccl::ShaderGraph &graph, ccl::Scene *scene), /* Parameters */
	graph, /* Object */
	scene /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(shader_graph, connect, /* namespace, function name */
	void, /* returnType */
	(ccl::ShaderGraph& graph, ccl::ShaderOutput *output, ccl::ShaderInput *input), /* Parameters */
	graph, /* Object */
	output COMMA input /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(shader_graph, dump_graph, /* namespace, function name */
	void, /* returnType */
	(ccl::ShaderGraph &graph, const char *filename), /* Parameters */
	graph, /* Object */
	filename /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(shader_graph, add, /* namespace, function name */
	ccl::ShaderNode*, /* returnType */
	(ccl::ShaderGraph &graph, ccl::ShaderNode *node), /* Parameters */
	graph, /* Object */
	node /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(shader_graph, output, /* namespace, function name */
	ccl::OutputNode*, /* returnType */
	(ccl::ShaderGraph &graph), /* Parameters */
	graph /* Object */
)

// socket_type
DEFINE_SIMPLE_FUNCTION(
	socket_type,set_name, /* namespace, function name */
	void, /* returnType */
	(ccl::SocketType& socketType, const char* name), /* Parameters */
	socketType COMMA name, /* Arguments */
	{
		socketType.name = name;
	}
)
DEFINE_SIMPLE_FUNCTION(
	socket_type,get_name, /* namespace, function name */
	void, /* returnType */
	(const ccl::SocketType& socketType, char **outStr, std::size_t& outStrLen), /* Parameters */
	socketType COMMA outStr COMMA outStrLen, /* Arguments */
	{
		return icycles::detail::str_to_cstr(socketType.name.c_str(),outStr,outStrLen);
	}
)

// shader
DEFINE_SIMPLE_FUNCTION(
	shader,create, /* namespace, function name */
	ccl::Shader*, /* returnType */
	(), /* Parameters */
	, /* Arguments */
	{
		return new ccl::Shader{};
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader,destroy, /* namespace, function name */
	void, /* returnType */
	(ccl::Shader *shader), /* Parameters */
	shader, /* Arguments */
	{
		icycles::detail::free(shader);
	}
)
DEFINE_SIMPLE_METHOD_N(shader, set_volume_sampling_method, /* namespace, function name */
	void, /* returnType */
	(ccl::Shader& shader, ccl::VolumeSampling sampling), /* Parameters */
	shader, /* Object */
	sampling /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(shader, set_graph, /* namespace, function name */
	void, /* returnType */
	(ccl::Shader& shader, ccl::ShaderGraph *graph), /* Parameters */
	shader, /* Object */
	graph /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(shader, tag_update, /* namespace, function name */
	void, /* returnType */
	(ccl::Shader& shader, ccl::Scene *scene), /* Parameters */
	shader, /* Object */
	scene /* Arguments */
)

// light
DEFINE_SIMPLE_FUNCTION(
	light,create, /* namespace, function name */
	ccl::Light*, /* returnType */
	(), /* Parameters */
	, /* Arguments */
	{
		return new ccl::Light{};
	}
)
DEFINE_SIMPLE_FUNCTION(
	light,destroy, /* namespace, function name */
	void, /* returnType */
	(ccl::Light *light), /* Parameters */
	light, /* Arguments */
	{
		icycles::detail::free(light);
	}
)
DEFINE_SIMPLE_METHOD_N(light, set_tfm, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, const ccl::Transform& value), /* Parameters */
	light, /* Object */
	value /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_light_type, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, ccl::LightType lightType), /* Parameters */
	light, /* Object */
	lightType /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_dir, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, const ccl::float3& dir), /* Parameters */
	light, /* Object */
	dir /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_spot_smooth, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, float spotSmooth), /* Parameters */
	light, /* Object */
	spotSmooth /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_spot_angle, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, float spotAngle), /* Parameters */
	light, /* Object */
	spotAngle /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_axisu, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, const ccl::float3& axis), /* Parameters */
	light, /* Object */
	axis /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_axisv, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, const ccl::float3& axis), /* Parameters */
	light, /* Object */
	axis /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_sizeu, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, float size), /* Parameters */
	light, /* Object */
	size /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_sizev, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, float size), /* Parameters */
	light, /* Object */
	size /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_round, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, bool round), /* Parameters */
	light, /* Object */
	round /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_strength, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, const ccl::float3& strength), /* Parameters */
	light, /* Object */
	strength /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_size, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, float size), /* Parameters */
	light, /* Object */
	size /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_co, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, const ccl::float3& co), /* Parameters */
	light, /* Object */
	co /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_max_bounces, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, int maxBounces), /* Parameters */
	light, /* Object */
	maxBounces /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_map_resolution, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, int mapResolution), /* Parameters */
	light, /* Object */
	mapResolution /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_use_mis, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, bool useMis), /* Parameters */
	light, /* Object */
	useMis /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, tag_update, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, ccl::Scene* scene), /* Parameters */
	light, /* Object */
	scene /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(light, set_shader, /* namespace, function name */
	void, /* returnType */
	(ccl::Light& light, ccl::Shader* shader), /* Parameters */
	light, /* Object */
	shader /* Arguments */
)

// camera
DEFINE_SIMPLE_METHOD_N(camera, set_camera_type, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, ccl::CameraType type), /* Parameters */
	cam, /* Object */
	type /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_panorama_type, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, ccl::PanoramaType type), /* Parameters */
	cam, /* Object */
	type /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_full_width, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, int width), /* Parameters */
	cam, /* Object */
	width /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_full_height, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, int height), /* Parameters */
	cam, /* Object */
	height /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_nearclip, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float nearClip), /* Parameters */
	cam, /* Object */
	nearClip /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_farclip, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float farClip), /* Parameters */
	cam, /* Object */
	farClip /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_fov, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float fov), /* Parameters */
	cam, /* Object */
	fov /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_focaldistance, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float focalDistance), /* Parameters */
	cam, /* Object */
	focalDistance /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_aperturesize, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float apertureSize), /* Parameters */
	cam, /* Object */
	apertureSize /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_aperture_ratio, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float apertureRatio), /* Parameters */
	cam, /* Object */
	apertureRatio /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_blades, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, uint32_t blades), /* Parameters */
	cam, /* Object */
	blades /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_bladesrotation, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float bladesRotation), /* Parameters */
	cam, /* Object */
	bladesRotation /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_interocular_distance, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float interocDist), /* Parameters */
	cam, /* Object */
	interocDist /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_longitude_max, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float longitudeMax), /* Parameters */
	cam, /* Object */
	longitudeMax /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_longitude_min, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float longitudeMin), /* Parameters */
	cam, /* Object */
	longitudeMin /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_latitude_max, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float latitudeMax), /* Parameters */
	cam, /* Object */
	latitudeMax /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_latitude_min, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float latitudeMin), /* Parameters */
	cam, /* Object */
	latitudeMin /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_use_spherical_stereo, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, bool useSphericalStereo), /* Parameters */
	cam, /* Object */
	useSphericalStereo /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_fisheye_lens, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float fisheyeLens), /* Parameters */
	cam, /* Object */
	fisheyeLens /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_fisheye_fov, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, float fisheyeFov), /* Parameters */
	cam, /* Object */
	fisheyeFov /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_stereo_eye, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, ccl::Camera::StereoEye eye), /* Parameters */
	cam, /* Object */
	eye /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(camera, set_matrix, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, const ccl::Transform& value), /* Parameters */
	cam, /* Object */
	value /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(camera, compute_auto_viewplane, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_camera_type, /* namespace, function name */
	ccl::CameraType, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_panorama_type, /* namespace, function name */
	ccl::PanoramaType, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_full_width, /* namespace, function name */
	int, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_full_height, /* namespace, function name */
	int, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_nearclip, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_farclip, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_fov, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_focaldistance, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_aperturesize, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_aperture_ratio, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_blades, /* namespace, function name */
	uint32_t, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_bladesrotation, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_interocular_distance, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_longitude_max, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_longitude_min, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_latitude_max, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_latitude_min, /* namespace, function name */
	float, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_use_spherical_stereo, /* namespace, function name */
	bool, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_0(camera, get_matrix, /* namespace, function name */
	const ccl::Transform&, /* returnType */
	(ccl::Camera& cam), /* Parameters */
	cam /* Object */
)
DEFINE_SIMPLE_METHOD_N(camera, update, /* namespace, function name */
	void, /* returnType */
	(ccl::Camera& cam, ccl::Scene* scene), /* Parameters */
	cam, /* Object */
	scene /* Arguments */
)
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	camera,need_flags_update, /* namespace, function name, member name */
	bool, /* value type */
	(ccl::Camera& cam), /* ParametersGetter */
	(ccl::Camera& cam,bool value), /* ParametersSetter */
	cam /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	camera,need_device_update, /* namespace, function name, member name */
	bool, /* value type */
	(ccl::Camera& cam), /* ParametersGetter */
	(ccl::Camera& cam,bool value), /* ParametersSetter */
	cam /* Object */
);

// mesh
DEFINE_SIMPLE_FUNCTION(
	mesh,create, /* namespace, function name */
	ccl::Mesh*, /* returnType */
	(), /* Parameters */
	, /* Arguments */
	{
		return new ccl::Mesh{};
	}
)
DEFINE_SIMPLE_FUNCTION(
	mesh,destroy, /* namespace, function name */
	void, /* returnType */
	(ccl::Mesh *mesh), /* Parameters */
	mesh, /* Arguments */
	{
		icycles::detail::free(mesh);
	}
)
DEFINE_SIMPLE_FUNCTION(
	mesh,set_verts, /* namespace, function name */
	void, /* returnType */
	(ccl::Mesh& mesh, ccl::array<ccl::float3>& verts), /* Parameters */
	mesh COMMA verts, /* Arguments */
	{
		mesh.set_verts(verts);
	}
)
DEFINE_SIMPLE_METHOD_0(mesh, get_triangles, /* namespace, function name */
	const ccl::array<int>&, /* returnType */
	(const ccl::Mesh& mesh), /* Parameters */
	mesh /* Object */
)
DEFINE_SIMPLE_METHOD_0(mesh, get_verts, /* namespace, function name */
	const ccl::array<ccl::float3>&, /* returnType */
	(const ccl::Mesh& mesh), /* Parameters */
	mesh /* Object */
)
DEFINE_SIMPLE_METHOD_N(mesh, reserve_mesh, /* namespace, function name */
	void, /* returnType */
	(ccl::Mesh& mesh, int numverts, int numfaces), /* Parameters */
	mesh, /* Object */
	numverts COMMA numfaces /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(mesh, add_vertex, /* namespace, function name */
	void, /* returnType */
	(ccl::Mesh& mesh, const ccl::float3& P), /* Parameters */
	mesh, /* Object */
	P /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(mesh, add_triangle, /* namespace, function name */
	void, /* returnType */
	(ccl::Mesh& mesh, int v0, int v1, int v2, int shader, bool smooth), /* Parameters */
	mesh, /* Object */
	v0 COMMA v1 COMMA v2 COMMA shader COMMA smooth /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(mesh, tag_update, /* namespace, function name */
	void, /* returnType */
	(ccl::Mesh& mesh, ccl::Scene *scene, bool rebuild), /* Parameters */
	mesh, /* Object */
	scene COMMA rebuild /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(mesh, get_num_subd_faces, /* namespace, function name */
	std::size_t, /* returnType */
	(const ccl::Mesh& mesh), /* Parameters */
	mesh /* Object */
)
DEFINE_SIMPLE_METHOD_0(mesh, num_triangles, /* namespace, function name */
	std::size_t, /* returnType */
	(const ccl::Mesh& mesh), /* Parameters */
	mesh /* Object */
)
DEFINE_SIMPLE_METHOD_N(mesh, get_subd_face, /* namespace, function name */
	ccl::Mesh::SubdFace, /* returnType */
	(const ccl::Mesh& mesh, std::size_t index), /* Parameters */
	mesh, /* Object */
	index /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(mesh, get_triangle, /* namespace, function name */
	ccl::Mesh::Triangle, /* returnType */
	(const ccl::Mesh& mesh, std::size_t index), /* Parameters */
	mesh, /* Object */
	index /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(mesh, get_smooth, /* namespace, function name */
	const ccl::array<bool>&, /* returnType */
	(const ccl::Mesh& mesh), /* Parameters */
	mesh /* Object */
)
DEFINE_SIMPLE_METHOD_0(mesh, get_subd_face_corners, /* namespace, function name */
	const ccl::array<int>&, /* returnType */
	(const ccl::Mesh& mesh), /* Parameters */
	mesh /* Object */
)
DEFINE_SIMPLE_MEMBER_GETTER(
	mesh,subd_attributes, /* namespace, member name */
	const ccl::AttributeSet&,
	(const ccl::Mesh& mesh), /* Parameters */
	mesh /* Object */
);

// object
DEFINE_SIMPLE_FUNCTION(
	object,create, /* namespace, function name */
	ccl::Object*, /* returnType */
	(), /* Parameters */
	, /* Arguments */
	{
		return new ccl::Object{};
	}
)
DEFINE_SIMPLE_FUNCTION(
	object,destroy, /* namespace, function name */
	void, /* returnType */
	(ccl::Object *object), /* Parameters */
	object, /* Arguments */
	{
		icycles::detail::free(object);
	}
)
DEFINE_SIMPLE_METHOD_N(object, set_tfm, /* namespace, function name */
	void, /* returnType */
	(ccl::Object& object, const ccl::Transform& value), /* Parameters */
	object, /* Object */
	value /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(object, set_geometry, /* namespace, function name */
	void, /* returnType */
	(ccl::Object& object, ccl::Geometry* geometry), /* Parameters */
	object, /* Object */
	geometry /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(object, get_geometry, /* namespace, function name */
	ccl::Geometry*, /* returnType */
	(ccl::Object& object), /* Parameters */
	object /* Object */
)
DEFINE_SIMPLE_METHOD_0(object, tag_tfm_modified, /* namespace, function name */
	void, /* returnType */
	(ccl::Object& object), /* Parameters */
	object /* Object */
)
DEFINE_SIMPLE_METHOD_N(object, tag_update, /* namespace, function name */
	void, /* returnType */
	(ccl::Object& object, ccl::Scene *scene), /* Parameters */
	object, /* Object */
	scene /* Arguments */
)

// geometry
DEFINE_SIMPLE_METHOD_N(geometry, apply_transform, /* namespace, function name */
	void, /* returnType */
	(ccl::Geometry& geo, const ccl::Transform& tfm, const bool apply_to_motion), /* Parameters */
	geo, /* Object */
	tfm COMMA apply_to_motion /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(geometry, get_used_shaders, /* namespace, function name */
	const ccl::array<ccl::Node*>&, /* returnType */
	(const ccl::Geometry& geo), /* Parameters */
	geo /* Object */
)
DEFINE_SIMPLE_METHOD_N(geometry, set_used_shaders, /* namespace, function name */
	void, /* returnType */
	(ccl::Geometry& geo, ccl::array<ccl::Node*>& shaders), /* Parameters */
	geo, /* Object */
	shaders /* Arguments */
)
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	geometry,transform_applied, /* namespace, function name, member name */
	bool, /* value type */
	(ccl::Geometry& geo), /* ParametersGetter */
	(ccl::Geometry& geo,bool value), /* ParametersSetter */
	geo /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	geometry,transform_normal, /* namespace, function name, member name */
	ccl::Transform, /* value type */
	(ccl::Geometry& geo), /* ParametersGetter */
	(ccl::Geometry& geo,const ccl::Transform &value), /* ParametersSetter */
	geo /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER(
	geometry,attributes, /* namespace, member name */
	ccl::AttributeSet&,
	(ccl::Geometry& geo), /* Parameters */
	geo /* Object */
);
DEFINE_SIMPLE_METHOD_N(geometry, need_attribute, /* namespace, function name */
	bool, /* returnType */
	(ccl::Geometry& geo, ccl::Scene *scene, ccl::AttributeStandard strd), /* Parameters */
	geo, /* Object */
	scene COMMA strd /* Arguments */
)

// pass
DEFINE_SIMPLE_METHOD_N(pass, set_type, /* namespace, function name */
	void, /* returnType */
	(ccl::Pass &pass, ccl::PassType type), /* Parameters */
	pass, /* Object */
	type /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(pass, set_include_albedo, /* namespace, function name */
	void, /* returnType */
	(ccl::Pass &pass, bool includeAlbedo), /* Parameters */
	pass, /* Object */
	includeAlbedo /* Arguments */
)

// session
DEFINE_SIMPLE_FUNCTION(
	session,set_display_driver, /* namespace, function name */
	void, /* returnType */
	(ccl::Session& session, ccl::DisplayDriver *driver), /* Parameters */
	session COMMA driver, /* Arguments */
	{
		session.set_display_driver(std::unique_ptr<ccl::DisplayDriver>{driver});
	}
)
DEFINE_SIMPLE_FUNCTION(
	session,set_output_driver, /* namespace, function name */
	void, /* returnType */
	(ccl::Session& session, ccl::OutputDriver *driver), /* Parameters */
	session COMMA driver, /* Arguments */
	{
		session.set_output_driver(std::unique_ptr<ccl::OutputDriver>{driver});
	}
)
DEFINE_SIMPLE_FUNCTION(
	session,create, /* namespace, function name */
	void, /* returnType */
	(const ccl::SessionParams &params, const ccl::SceneParams &scene_params, icycles::Session &outSession), /* Parameters */
	params COMMA scene_params COMMA outSession, /* Arguments */
	{
		outSession = icycles::Session{new ccl::Session{params COMMA scene_params} COMMA [](ccl::Session *ptr) {
			icycles::detail::free(ptr);
		}};
	}
)
DEFINE_SIMPLE_METHOD_0(session, start, /* namespace, function name */
	void, /* returnType */
	(ccl::Session& session), /* Parameters */
	session /* Object */
)
DEFINE_SIMPLE_METHOD_N(session, set_pause, /* namespace, function name */
	void, /* returnType */
	(ccl::Session& session, bool pause), /* Parameters */
	session, /* Object */
	pause /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(session, cancel, /* namespace, function name */
	void, /* returnType */
	(ccl::Session& session, bool quick), /* Parameters */
	session, /* Object */
	quick /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(session, wait, /* namespace, function name */
	void, /* returnType */
	(ccl::Session& session), /* Parameters */
	session /* Object */
)
DEFINE_SIMPLE_MEMBER_GETTER(
	session,progress, /* namespace, member name */
	ccl::Progress&,
	(ccl::Session& session), /* Parameters */
	session /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER(
	session,params, /* namespace, member name */
	ccl::SessionParams&,
	(ccl::Session& session), /* Parameters */
	session /* Object */
);
DEFINE_SIMPLE_METHOD_N(session, reset, /* namespace, function name */
	void, /* returnType */
	(ccl::Session& session, const ccl::SessionParams &session_params, const ccl::BufferParams &buffer_params), /* Parameters */
	session, /* Object */
	session_params COMMA buffer_params /* Arguments */
)
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	session,scene, /* namespace, function name, member name */
	ccl::Scene*,
	(ccl::Session& session), /* ParametersGetter */
	(ccl::Session& session,ccl::Scene *value), /* ParametersSetter */
	session /* Object */
);

// scene
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene,dicing_camera, /* namespace, function name, member name */
	ccl::Camera*,
	(ccl::Scene& scene), /* ParametersGetter */
	(ccl::Scene& scene,ccl::Camera *value), /* ParametersSetter */
	scene /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene,integrator, /* namespace, function name, member name */
	ccl::Integrator*,
	(ccl::Scene& scene), /* ParametersGetter */
	(ccl::Scene& scene,ccl::Integrator *value), /* ParametersSetter */
	scene /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene,camera, /* namespace, function name, member name */
	ccl::Camera*,
	(ccl::Scene& scene), /* ParametersGetter */
	(ccl::Scene& scene,ccl::Camera *value), /* ParametersSetter */
	scene /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene,bake_manager, /* namespace, function name, member name */
	ccl::BakeManager*,
	(ccl::Scene& scene), /* ParametersGetter */
	(ccl::Scene& scene,ccl::BakeManager *value), /* ParametersSetter */
	scene /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene,film, /* namespace, function name, member name */
	ccl::Film*,
	(ccl::Scene& scene), /* ParametersGetter */
	(ccl::Scene& scene,ccl::Film *value), /* ParametersSetter */
	scene /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene,background, /* namespace, function name, member name */
	ccl::Background*,
	(ccl::Scene& scene), /* ParametersGetter */
	(ccl::Scene& scene,ccl::Background *value), /* ParametersSetter */
	scene /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene,default_background, /* namespace, function name, member name */
	ccl::Shader*,
	(ccl::Scene& scene), /* ParametersGetter */
	(ccl::Scene& scene,ccl::Shader *value), /* ParametersSetter */
	scene /* Object */
);
DEFINE_SIMPLE_FUNCTION(
	scene,create_node_pass, /* namespace, function name */
	ccl::Pass*, /* returnType */
	(ccl::Scene& scene), /* Parameters */
	scene, /* Arguments */
	{
		return scene.create_node<ccl::Pass>();
	}
)
DEFINE_SIMPLE_FUNCTION(
	scene,add_geometry, /* namespace, function name */
	void, /* returnType */
	(ccl::Scene& scene, ccl::Geometry *geometry), /* Parameters */
	scene COMMA geometry, /* Arguments */
	{
		scene.geometry.push_back(geometry);
	}
)

// film
DEFINE_SIMPLE_METHOD_N(film, set_pass_alpha_threshold, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, float threshold), /* Parameters */
	film, /* Object */
	threshold /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_exposure, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, float exposure), /* Parameters */
	film, /* Object */
	exposure /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_filter_type, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, ccl::FilterType filterType), /* Parameters */
	film, /* Object */
	filterType /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_filter_width, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, float width), /* Parameters */
	film, /* Object */
	width /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_mist_start, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, float start), /* Parameters */
	film, /* Object */
	start /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_mist_depth, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, float depth), /* Parameters */
	film, /* Object */
	depth /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_mist_falloff, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, float falloff), /* Parameters */
	film, /* Object */
	falloff /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_cryptomatte_depth, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, float cryptoDept), /* Parameters */
	film, /* Object */
	cryptoDept /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(film, set_cryptomatte_passes, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film, ccl::CryptomatteType type), /* Parameters */
	film, /* Object */
	type /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(film, tag_modified, /* namespace, function name */
	void, /* returnType */
	(ccl::Film& film), /* Parameters */
	film /* Object */
)

// integrator
DEFINE_SIMPLE_METHOD_N(integrator, set_motion_blur, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool motionBlur), /* Parameters */
	integrator, /* Object */
	motionBlur /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_min_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int minBounce), /* Parameters */
	integrator, /* Object */
	minBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_max_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int maxBounce), /* Parameters */
	integrator, /* Object */
	maxBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_max_diffuse_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int maxDiffBounce), /* Parameters */
	integrator, /* Object */
	maxDiffBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_max_glossy_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int maxGlossBounce), /* Parameters */
	integrator, /* Object */
	maxGlossBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_max_transmission_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int maxTransBounce), /* Parameters */
	integrator, /* Object */
	maxTransBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_max_volume_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int maxVolBounce), /* Parameters */
	integrator, /* Object */
	maxVolBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_transparent_min_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int minBounce), /* Parameters */
	integrator, /* Object */
	minBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_transparent_max_bounce, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int maxBounce), /* Parameters */
	integrator, /* Object */
	maxBounce /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_volume_max_steps, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int maxSteps), /* Parameters */
	integrator, /* Object */
	maxSteps /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_volume_step_rate, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, float stepRate), /* Parameters */
	integrator, /* Object */
	stepRate /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_caustics_reflective, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool reflective), /* Parameters */
	integrator, /* Object */
	reflective /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_caustics_refractive, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool refractive), /* Parameters */
	integrator, /* Object */
	refractive /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_filter_glossy, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, float filterGlossy), /* Parameters */
	integrator, /* Object */
	filterGlossy /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_seed, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int seed), /* Parameters */
	integrator, /* Object */
	seed /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_sampling_pattern, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, ccl::SamplingPattern pattern), /* Parameters */
	integrator, /* Object */
	pattern /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_use_adaptive_sampling, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool useAdaptiveSampling), /* Parameters */
	integrator, /* Object */
	useAdaptiveSampling /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_adaptive_threshold, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, float threshold), /* Parameters */
	integrator, /* Object */
	threshold /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_adaptive_min_samples, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int minSamples), /* Parameters */
	integrator, /* Object */
	minSamples /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(integrator, tag_modified, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator), /* Parameters */
	integrator /* Object */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_use_denoise, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool denoise), /* Parameters */
	integrator, /* Object */
	denoise /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_denoiser_type, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, ccl::DenoiserType type), /* Parameters */
	integrator, /* Object */
	type /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_denoise_start_sample, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, int startSample), /* Parameters */
	integrator, /* Object */
	startSample /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_use_denoise_pass_albedo, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool usePassAlbedo), /* Parameters */
	integrator, /* Object */
	usePassAlbedo /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_use_denoise_pass_normal, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool usePassNormal), /* Parameters */
	integrator, /* Object */
	usePassNormal /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_use_direct_light, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool use), /* Parameters */
	integrator, /* Object */
	use /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(integrator, set_use_indirect_light, /* namespace, function name */
	void, /* returnType */
	(ccl::Integrator& integrator, bool use), /* Parameters */
	integrator, /* Object */
	use /* Arguments */
)

// background
DEFINE_SIMPLE_METHOD_N(background, set_transparent, /* namespace, function name */
	void, /* returnType */
	(ccl::Background& integrator, bool transparent), /* Parameters */
	integrator, /* Object */
	transparent /* Arguments */
)

// hair
DEFINE_SIMPLE_FUNCTION(
	hair,create, /* namespace, function name */
	ccl::Hair*, /* returnType */
	(), /* Parameters */
	, /* Arguments */
	{
		return new ccl::Hair{};
	}
)
DEFINE_SIMPLE_FUNCTION(
	hair,destroy, /* namespace, function name */
	void, /* returnType */
	(ccl::Hair *hair), /* Parameters */
	hair, /* Arguments */
	{
		icycles::detail::free(hair);
	}
)
DEFINE_SIMPLE_METHOD_N(hair, reserve_curves, /* namespace, function name */
	void, /* returnType */
	(ccl::Hair& hair, int numcurves, int numkeys), /* Parameters */
	hair, /* Object */
	numcurves COMMA numkeys /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(hair, add_curve, /* namespace, function name */
	void, /* returnType */
	(ccl::Hair& hair, int first_key, int shader), /* Parameters */
	hair, /* Object */
	first_key COMMA shader /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(hair, add_curve_key, /* namespace, function name */
	void, /* returnType */
	(ccl::Hair& hair, ccl::float3 loc, float radius), /* Parameters */
	hair, /* Object */
	loc COMMA radius /* Arguments */
)

// progress
DEFINE_SIMPLE_FUNCTION(
	progress,get_cancel_message, /* namespace, function name */
	void, /* returnType */
	(ccl::Progress& progress, char **outStr, std::size_t& outStrLen), /* Parameters */
	progress COMMA outStr COMMA outStrLen, /* Arguments */
	{
		return icycles::detail::str_to_cstr(progress.get_cancel_message(),outStr,outStrLen);
	}
)
DEFINE_SIMPLE_FUNCTION(
	progress,get_error_message, /* namespace, function name */
	void, /* returnType */
	(ccl::Progress& progress, char **outStr, std::size_t& outStrLen), /* Parameters */
	progress COMMA outStr COMMA outStrLen, /* Arguments */
	{
		return icycles::detail::str_to_cstr(progress.get_error_message(),outStr,outStrLen);
	}
)
DEFINE_SIMPLE_FUNCTION(
	progress,get_progress_status, /* namespace, function name */
	void, /* returnType */
	(ccl::Progress& progress, char **outStrStatus, std::size_t& outStrStatusLen, char **outStrSubStatus, std::size_t& outStrSubStatusLen), /* Parameters */
	progress COMMA outStrStatus COMMA outStrStatusLen COMMA outStrSubStatus COMMA outStrSubStatusLen, /* Arguments */
	{
		std::string status;
		std::string subStatus;
		progress.get_status(status,subStatus);
		icycles::detail::str_to_cstr(status,outStrStatus,outStrStatusLen);
		icycles::detail::str_to_cstr(subStatus,outStrSubStatus,outStrSubStatusLen);
	}
)
DEFINE_SIMPLE_METHOD_0(progress, reset, /* namespace, function name */
	void, /* returnType */
	(ccl::Progress& progress), /* Parameters */
	progress /* Object */
)
DEFINE_SIMPLE_METHOD_N(progress, set_cancel, /* namespace, function name */
	void, /* returnType */
	(ccl::Progress& progress, const char* msg), /* Parameters */
	progress, /* Object */
	msg /* Arguments */
)
DEFINE_SIMPLE_METHOD_0(progress, get_progress, /* namespace, function name */
	double, /* returnType */
	(ccl::Progress& progress), /* Parameters */
	progress /* Object */
)

// shader_input
DEFINE_SIMPLE_FUNCTION(
	shader_input,get_name, /* namespace, function name */
	void, /* returnType */
	(ccl::ShaderInput& input, char **outStr, std::size_t& outStrLen), /* Parameters */
	input COMMA outStr COMMA outStrLen, /* Arguments */
	{
		icycles::detail::str_to_cstr(input.name().c_str(),outStr,outStrLen);
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_input,get_socket_type, /* namespace, function name */
	const ccl::SocketType&, /* returnType */
	(ccl::ShaderInput& input), /* Parameters */
	input, /* Arguments */
	{
		return input.socket_type;
	}
)

// shader_output
DEFINE_SIMPLE_FUNCTION(
	shader_output,get_name, /* namespace, function name */
	void, /* returnType */
	(ccl::ShaderOutput& output, char **outStr, std::size_t& outStrLen), /* Parameters */
	output COMMA outStr COMMA outStrLen, /* Arguments */
	{
		icycles::detail::str_to_cstr(output.name().c_str(),outStr,outStrLen);
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_output,get_socket_type, /* namespace, function name */
	const ccl::SocketType&, /* returnType */
	(ccl::ShaderOutput& input), /* Parameters */
	input, /* Arguments */
	{
		return input.socket_type;
	}
)

// shader_node
DEFINE_SIMPLE_FUNCTION(
	shader_node,get_type_enum, /* namespace, function name */
	icycles::ShaderNodeType, /* returnType */
	(const ccl::ShaderNode& node), /* Parameters */
	node, /* Arguments */
	{
		auto &t = typeid(node);
		if(t == typeid(ccl::MathNode))
			return icycles::ShaderNodeType::Math;
		else if(t == typeid(ccl::MappingNode))
			return icycles::ShaderNodeType::Mapping;
		else if(t == typeid(ccl::MixNode))
			return icycles::ShaderNodeType::Mix;
		else if(t == typeid(ccl::VectorMathNode))
			return icycles::ShaderNodeType::VectorMath;
		else if(t == typeid(ccl::VectorTransformNode))
			return icycles::ShaderNodeType::VectorTransform;
		else if(t == typeid(ccl::ImageTextureNode))
			return icycles::ShaderNodeType::ImageTexture;
		else if(t == typeid(ccl::NormalMapNode))
			return icycles::ShaderNodeType::NormalMap;
		return icycles::ShaderNodeType::Other;
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_node,get_input_count, /* namespace, function name */
	std::size_t, /* returnType */
	(ccl::ShaderNode& node), /* Parameters */
	node, /* Arguments */
	{
		return node.inputs.size();
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_node,get_output_count, /* namespace, function name */
	std::size_t, /* returnType */
	(ccl::ShaderNode& node), /* Parameters */
	node, /* Arguments */
	{
		return node.outputs.size();
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_node,get_input, /* namespace, function name */
	ccl::ShaderInput*, /* returnType */
	(ccl::ShaderNode& node, std::size_t index), /* Parameters */
	node COMMA index, /* Arguments */
	{
		return node.inputs[index];
	}
)
DEFINE_SIMPLE_FUNCTION(
	shader_node,get_output, /* namespace, function name */
	ccl::ShaderOutput*, /* returnType */
	(ccl::ShaderNode& node, std::size_t index), /* Parameters */
	node COMMA index, /* Arguments */
	{
		return node.outputs[index];
	}
)
DEFINE_SIMPLE_METHOD_N(shader_node, set_owner, /* namespace, function name */
	void, /* returnType */
	(ccl::ShaderNode& node, const ccl::NodeOwner* owner), /* Parameters */
	node, /* Object */
	owner /* Arguments */
)
DEFINE_SIMPLE_MEMBER_GETTER(
	shader_node,type, /* namespace, member name */
	const ccl::NodeType*, /* value type */
	(const ccl::ShaderNode& node), /* Parameters */
	node /* Object */
);

// math_node
DEFINE_SIMPLE_METHOD_N(math_node, set_math_type, /* namespace, function name */
	void, /* returnType */
	(ccl::MathNode& node, ccl::NodeMathType type), /* Parameters */
	node, /* Object */
	type /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(math_node, set_value1, /* namespace, function name */
	void, /* returnType */
	(ccl::MathNode& node, float value), /* Parameters */
	node, /* Object */
	value /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(math_node, set_value2, /* namespace, function name */
	void, /* returnType */
	(ccl::MathNode& node, float value), /* Parameters */
	node, /* Object */
	value /* Arguments */
)

// vector_math_node
DEFINE_SIMPLE_METHOD_N(vector_math_node, set_math_type, /* namespace, function name */
	void, /* returnType */
	(ccl::VectorMathNode& node, ccl::NodeVectorMathType type), /* Parameters */
	node, /* Object */
	type /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(math_node, set_vector1, /* namespace, function name */
	void, /* returnType */
	(ccl::VectorMathNode& node, const ccl::float3 &value), /* Parameters */
	node, /* Object */
	value /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(math_node, set_vector2, /* namespace, function name */
	void, /* returnType */
	(ccl::VectorMathNode& node, const ccl::float3 &value), /* Parameters */
	node, /* Object */
	value /* Arguments */
)

// mix_closure_node
DEFINE_SIMPLE_METHOD_N(mix_closure_node, set_fac, /* namespace, function name */
	void, /* returnType */
	(ccl::MixClosureNode& node, float factor), /* Parameters */
	node, /* Object */
	factor /* Arguments */
)

// emission_node
DEFINE_SIMPLE_METHOD_N(emission_node, set_color, /* namespace, function name */
	void, /* returnType */
	(ccl::EmissionNode& node, const ccl::float3 &color), /* Parameters */
	node, /* Object */
	color /* Arguments */
)

// sky_texture_node
DEFINE_SIMPLE_METHOD_N(sky_texture_node, set_sky_type, /* namespace, function name */
	void, /* returnType */
	(ccl::SkyTextureNode& node, ccl::NodeSkyType type), /* Parameters */
	node, /* Object */
	type /* Arguments */
)

// background_node
DEFINE_SIMPLE_METHOD_N(background_node, set_strength, /* namespace, function name */
	void, /* returnType */
	(ccl::BackgroundNode& node, float strength), /* Parameters */
	node, /* Object */
	strength /* Arguments */
)
DEFINE_SIMPLE_METHOD_N(background_node, set_color, /* namespace, function name */
	void, /* returnType */
	(ccl::BackgroundNode& node, const ccl::float3 &color), /* Parameters */
	node, /* Object */
	color /* Arguments */
)

// node
DEFINE_SIMPLE_FUNCTION(
	node,set_name, /* namespace, function name */
	void, /* returnType */
	(ccl::Node& node, const char* name), /* Parameters */
	node COMMA name, /* Arguments */
	{
		node.name = name;
	}
)
DEFINE_SIMPLE_FUNCTION(
	node,get_name, /* namespace, function name */
	void, /* returnType */
	(ccl::Node& node, const char* name), /* Parameters */
	node COMMA name, /* Arguments */
	{
		node.name = name;
	}
)
DEFINE_SIMPLE_MEMBER_GETTER(
	node,type, /* namespace, member name */
	const ccl::NodeType*,
	(ccl::Node& node), /* Parameters */
	node /* Object */
);
#define DEFINE_NODE_SET(funcName,type) \
	DEFINE_SIMPLE_FUNCTION( \
		node,funcName, \
		void, \
		(ccl::Node& node, const ccl::SocketType &input, type value), \
		node COMMA input COMMA value, \
		{ \
			node.set(input,value); \
		} \
	)
DEFINE_NODE_SET(set_boolean,bool)
DEFINE_NODE_SET(set_int,int)
DEFINE_NODE_SET(set_uint,uint32_t)
DEFINE_NODE_SET(set_float,float)
DEFINE_NODE_SET(set_float2,ccl::float2)
DEFINE_NODE_SET(set_float3,ccl::float3)
DEFINE_NODE_SET(set_cstring,const char*)
DEFINE_NODE_SET(set_transform,const ccl::Transform&)
DEFINE_NODE_SET(set_node,ccl::Node*)

DEFINE_NODE_SET(set_boolean_a,ccl::array<bool>&)
DEFINE_NODE_SET(set_float_a,ccl::array<float>&)
DEFINE_NODE_SET(set_float2_a,ccl::array<ccl::float2>&)
DEFINE_NODE_SET(set_float3_a,ccl::array<ccl::float3>&)
DEFINE_NODE_SET(set_transform_a,ccl::array<ccl::Transform>&)
DEFINE_NODE_SET(set_node_a,ccl::array<ccl::Node*>&)

// image_texture_node
DEFINE_SIMPLE_FUNCTION(
	image_texture_node,set_colorspace, /* namespace, function name */
	void, /* returnType */
	(ccl::ImageTextureNode& node, const char* colorSpace), /* Parameters */
	node COMMA colorSpace, /* Arguments */
	{
		node.set_colorspace(ccl::ustring{colorSpace});
	}
)
	
// normal_map_node
DEFINE_SIMPLE_FUNCTION(
	normal_map_node,set_space, /* namespace, function name */
	void, /* returnType */
	(ccl::NormalMapNode& node, ccl::NodeNormalMapSpace space), /* Parameters */
	node COMMA space, /* Arguments */
	{
		node.set_space(space);
	}
)

// node_type
DEFINE_SIMPLE_FUNCTION(
	node_type,get_input_count, /* namespace, function name */
	std::size_t, /* returnType */
	(const ccl::NodeType &nodeType), /* Parameters */
	nodeType, /* Arguments */
	{
		return nodeType.inputs.size();
	}
)
DEFINE_SIMPLE_FUNCTION(
	node_type,get_input, /* namespace, function name */
	const ccl::SocketType*, /* returnType */
	(const ccl::NodeType &nodeType, size_t index), /* Parameters */
	nodeType COMMA index, /* Arguments */
	{
		return &nodeType.inputs[index];
	}
)
DEFINE_SIMPLE_FUNCTION(
	node_type,create_node, /* namespace, function name */
	ccl::Node*, /* returnType */
	(const ccl::NodeType &nodeType), /* Parameters */
	nodeType, /* Arguments */
	{
		return nodeType.create(&nodeType);
	}
)
DEFINE_SIMPLE_FUNCTION(
	node_type,find_input, /* namespace, function name */
	const ccl::SocketType*, /* returnType */
	(const ccl::NodeType &nodeType, const char *str), /* Parameters */
	nodeType COMMA str, /* Arguments */
	{
		return nodeType.find_input(ccl::ustring{str});
	}
)
DEFINE_SIMPLE_FUNCTION(
	node_type,find_output, /* namespace, function name */
	const ccl::SocketType*, /* returnType */
	(const ccl::NodeType &nodeType, const char *str), /* Parameters */
	nodeType COMMA str, /* Arguments */
	{
		return nodeType.find_output(ccl::ustring{str});
	}
)

// transform
DEFINE_SIMPLE_FUNCTION(
	transform,transposed_inverse, /* namespace, function name */
	ccl::Transform, /* returnType */
	(const ccl::Transform &transform), /* Parameters */
	transform, /* Arguments */
	{
		return ccl::transform_transposed_inverse(transform);
	}
)

// triangle
DEFINE_SIMPLE_FUNCTION(
	triangle,compute_normal, /* namespace, function name */
	ccl::float3, /* returnType */
	(const ccl::Mesh::Triangle &tri, const ccl::float3 *verts), /* Parameters */
	tri COMMA verts, /* Arguments */
	{
		return tri.compute_normal(verts);
	}
)

// subd_face
DEFINE_SIMPLE_FUNCTION(
	subd_face,normal, /* namespace, function name */
	ccl::float3, /* returnType */
	(const ccl::Mesh::SubdFace &face, const ccl::Mesh *mesh), /* Parameters */
	face COMMA mesh, /* Arguments */
	{
		return face.normal(mesh);
	}
)

// device_info
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	device_info,type, /* namespace, function name, member name */
	ccl::DeviceType, /* value type */
	(ccl::DeviceInfo& deviceInfo), /* ParametersGetter */
	(ccl::DeviceInfo& deviceInfo,ccl::DeviceType value), /* ParametersSetter */
	deviceInfo /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	device_info,denoisers, /* namespace, function name, member name */
	ccl::DenoiserTypeMask, /* value type */
	(ccl::DeviceInfo& deviceInfo), /* ParametersGetter */
	(ccl::DeviceInfo& deviceInfo,ccl::DenoiserTypeMask value), /* ParametersSetter */
	deviceInfo /* Object */
);

// device
DEFINE_SIMPLE_FUNCTION(
	device,type_from_string, /* namespace, function name */
	ccl::DeviceType, /* returnType */
	(const char *name), /* Parameters */
	name, /* Arguments */
	{
		return ccl::Device::type_from_string(name);
	}
)
DEFINE_SIMPLE_FUNCTION(
	device,string_from_type, /* namespace, function name */
	void, /* returnType */
	(ccl::DeviceType deviceType, char **outStr, size_t& outStrLen), /* Parameters */
	deviceType COMMA outStr COMMA outStrLen, /* Arguments */
	{
		icycles::detail::str_to_cstr(ccl::Device::string_from_type(deviceType).c_str(),outStr,outStrLen);
	}
)
DEFINE_SIMPLE_FUNCTION(
	device,available_types, /* namespace, function name */
	void, /* returnType */
	(std::vector<ccl::DeviceType> &outTypes), /* Parameters */
	outTypes, /* Arguments */
	{
		outTypes = ccl::Device::available_types();
	}
)
DEFINE_SIMPLE_FUNCTION(
	device,is_type_available, /* namespace, function name */
	bool, /* returnType */
	(ccl::DeviceType type), /* Parameters */
	type, /* Arguments */
	{
		using namespace ccl;
		return ccl::Device::available_devices(DEVICE_MASK(type)).empty() == false;
	}
)

DEFINE_SIMPLE_FUNCTION(
	device,get_available_devices, /* namespace, function name */
	void, /* returnType */
	(uint32_t device_type_mask,std::vector<std::shared_ptr<ccl::DeviceInfo>> &outDevices), /* Parameters */
	device_type_mask COMMA outDevices, /* Arguments */
	{
		std::vector<std::shared_ptr<ccl::DeviceInfo>> types;
		auto devices = ccl::Device::available_devices(device_type_mask);
		types.reserve(devices.size());
		for(auto &devInfo : devices)
			types.push_back(std::make_shared<ccl::DeviceInfo>(devInfo));
		outDevices = std::move(types);
	}
)

// bake_manager
DEFINE_SIMPLE_FUNCTION(
	bake_manager,set, /* namespace, function name */
	void, /* returnType */
	(ccl::BakeManager &bakeManager, ccl::Scene *scene, const char *object_name), /* Parameters */
	bakeManager COMMA scene COMMA object_name, /* Arguments */
	{
		bakeManager.set(scene,object_name);
	}
)

// scene_params
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	scene_params,shadingsystem, /* namespace, function name, member name */
	ccl::ShadingSystem, /* value type */
	(ccl::SceneParams& sceneParams), /* ParametersGetter */
	(ccl::SceneParams& sceneParams,ccl::ShadingSystem value), /* ParametersSetter */
	sceneParams /* Object */
);

// buffer_params
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	buffer_params,width, /* namespace, function name, member name */
	int, /* value type */
	(ccl::BufferParams& bufferParams), /* ParametersGetter */
	(ccl::BufferParams& bufferParams,int value), /* ParametersSetter */
	bufferParams /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	buffer_params,height, /* namespace, function name, member name */
	int, /* value type */
	(ccl::BufferParams& bufferParams), /* ParametersGetter */
	(ccl::BufferParams& bufferParams,int value), /* ParametersSetter */
	bufferParams /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	buffer_params,full_width, /* namespace, function name, member name */
	int, /* value type */
	(ccl::BufferParams& bufferParams), /* ParametersGetter */
	(ccl::BufferParams& bufferParams,int value), /* ParametersSetter */
	bufferParams /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	buffer_params,full_height, /* namespace, function name, member name */
	int, /* value type */
	(ccl::BufferParams& bufferParams), /* ParametersGetter */
	(ccl::BufferParams& bufferParams,int value), /* ParametersSetter */
	bufferParams /* Object */
);
DEFINE_SIMPLE_FUNCTION(
	buffer_params,copy, /* namespace, function name */
	void, /* returnType */
	(const ccl::BufferParams& srcBufferParams, ccl::BufferParams& dstBufferParams), /* Parameters */
	srcBufferParams COMMA dstBufferParams, /* Arguments */
	{
		dstBufferParams = srcBufferParams;
	}
)

// session_params
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	session_params,shadingsystem, /* namespace, function name, member name */
	ccl::ShadingSystem, /* value type */
	(ccl::SessionParams& sessionParams), /* ParametersGetter */
	(ccl::SessionParams& sessionParams,ccl::ShadingSystem value), /* ParametersSetter */
	sessionParams /* Object */
);
DEFINE_SIMPLE_FUNCTION(
	session_params,get_device, /* namespace, function name */
	void, /* returnType */
	(ccl::SessionParams& sessionParams,ccl::DeviceInfo &outDeviceInfo), /* Parameters */
	sessionParams COMMA outDeviceInfo, /* Arguments */
	{
		outDeviceInfo = sessionParams.device;
	}
)
DEFINE_SIMPLE_MEMBER_SETTER(
	session_params,device, /* namespace, function name, member name */
	(ccl::SessionParams& sessionParams,ccl::DeviceInfo value), /* Parameters */
	sessionParams /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	session_params,background, /* namespace, function name, member name */
	bool, /* value type */
	(ccl::SessionParams& sessionParams), /* ParametersGetter */
	(ccl::SessionParams& sessionParams,bool value), /* ParameterSetter */
	sessionParams /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	session_params,use_auto_tile, /* namespace, function name, member name */
	bool, /* value type */
	(ccl::SessionParams& sessionParams), /* ParametersGetter */
	(ccl::SessionParams& sessionParams,bool value), /* ParameterSetter */
	sessionParams /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	session_params,tile_size, /* namespace, function name, member name */
	int, /* value type */
	(ccl::SessionParams& sessionParams), /* ParametersGetter */
	(ccl::SessionParams& sessionParams,int value), /* ParameterSetter */
	sessionParams /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER_AND_SETTER(
	session_params,samples, /* namespace, function name, member name */
	int, /* value type */
	(ccl::SessionParams& sessionParams), /* ParametersGetter */
	(ccl::SessionParams& sessionParams,int value), /* ParameterSetter */
	sessionParams /* Object */
);
DEFINE_SIMPLE_FUNCTION(
	session_params,copy, /* namespace, function name */
	void, /* returnType */
	(const ccl::SessionParams& srcSessionParams, ccl::SessionParams& dstSessionParams), /* Parameters */
	srcSessionParams COMMA dstSessionParams, /* Arguments */
	{
		dstSessionParams = srcSessionParams;
	}
)
DEFINE_SIMPLE_FUNCTION(
	session_params,set_device_by_type, /* namespace, function name */
	void, /* returnType */
	(ccl::SessionParams &sessionParams, ccl::DeviceType type), /* Parameters */
	sessionParams COMMA type, /* Arguments */
	{
		using namespace ccl;
		auto devices = ccl::Device::available_devices(DEVICE_MASK(type));
		if(!devices.empty())
			sessionParams.device = devices.front();
	}
)

// output_driver
DEFINE_SIMPLE_FUNCTION(
	output_driver,create, /* namespace, function name */
	ccl::OutputDriver*, /* returnType */
	(std::shared_ptr<void> userData, void(*writeRenderTile)(void* COMMA const ccl::OutputDriver::Tile&), bool(*updateRenderTile)(void* COMMA const ccl::OutputDriver::Tile&), bool(*readRenderTile)(void* COMMA const ccl::OutputDriver::Tile&)), /* Parameters */
	userData COMMA writeRenderTile COMMA updateRenderTile COMMA readRenderTile, /* Arguments */
	{
		auto driver = new icycles::impl::BaseOutputDriver{};
		driver->Setup(userData,writeRenderTile,updateRenderTile,readRenderTile);
		return driver;
	}
)
DEFINE_SIMPLE_FUNCTION(
	output_driver,free, /* namespace, function name */
	void, /* returnType */
	(ccl::OutputDriver *ptr), /* Parameters */
	ptr, /* Arguments */
	{
		icycles::detail::free(ptr);
	}
)
DEFINE_SIMPLE_FUNCTION(
	output_driver_tile,get_pass_pixels, /* namespace, function name */
	bool, /* returnType */
	(const ccl::OutputDriver::Tile &tile, const char *pass_name, const int num_channels, float *pixels), /* Parameters */
	tile COMMA pass_name COMMA num_channels COMMA pixels, /* Arguments */
	{
		return tile.get_pass_pixels(pass_name,num_channels,pixels);
	}
)
DEFINE_SIMPLE_FUNCTION(
	output_driver_tile,set_pass_pixels, /* namespace, function name */
	bool, /* returnType */
	(const ccl::OutputDriver::Tile &tile, const char *pass_name, const int num_channels, const float *pixels), /* Parameters */
	tile COMMA pass_name COMMA num_channels COMMA pixels, /* Arguments */
	{
		return tile.set_pass_pixels(pass_name,num_channels,pixels);
	}
)
DEFINE_SIMPLE_MEMBER_GETTER(
	output_driver_tile,size, /* namespace, function name, member name */
	const ccl::int2&, /* value type */
	(ccl::OutputDriver::Tile &tile), /* Parameters */
	tile /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER(
	output_driver_tile,full_size, /* namespace, function name, member name */
	const ccl::int2&, /* value type */
	(ccl::OutputDriver::Tile &tile), /* Parameters */
	tile /* Object */
);
DEFINE_SIMPLE_MEMBER_GETTER(
	output_driver_tile,offset, /* namespace, function name, member name */
	const ccl::int2&, /* value type */
	(ccl::OutputDriver::Tile &tile), /* Parameters */
	tile /* Object */
);

// image_output
DEFINE_SIMPLE_FUNCTION(
	image_output,create, /* namespace, function name */
	void, /* returnType */
	(const char *filepath,std::shared_ptr<ccl::ImageOutput> &outImageOutput), /* Parameters */
	filepath COMMA outImageOutput, /* Arguments */
	{
		outImageOutput = std::shared_ptr<ccl::ImageOutput>{ccl::ImageOutput::create(filepath)};
	}
)
DEFINE_SIMPLE_FUNCTION(
	image_output,open, /* namespace, function name */
	bool, /* returnType */
	(ccl::ImageOutput &imgOutput, const char *filepath, ccl::ImageSpec &imgSpec), /* Parameters */
	imgOutput COMMA filepath COMMA imgSpec, /* Arguments */
	{
		return imgOutput.open(filepath,imgSpec);
	}
)
DEFINE_SIMPLE_FUNCTION(
	image_output,write_image, /* namespace, function name */
	bool, /* returnType */
	(ccl::ImageOutput &imgOutput, ccl::TypeDesc typeDesc, const void *data, int64_t xstride, int64_t ystride, int64_t zstride), /* Parameters */
	imgOutput COMMA typeDesc COMMA data COMMA xstride COMMA ystride COMMA zstride, /* Arguments */
	{
		return imgOutput.write_image(typeDesc,data,xstride,ystride,zstride);
	}
)
DEFINE_SIMPLE_FUNCTION(
	image_output,close, /* namespace, function name */
	bool, /* returnType */
	(ccl::ImageOutput &imgOutput), /* Parameters */
	imgOutput, /* Arguments */
	{
		return imgOutput.close();
	}
)

// global
DEFINE_SIMPLE_FUNCTION(
	util,create_image_spec, /* namespace, function name */
	void, /* returnType */
	(int xres, int yres, int nchans, ccl::TypeDesc fmt,std::shared_ptr<ccl::ImageSpec> &outImageSpec), /* Parameters */
	xres COMMA yres COMMA nchans COMMA fmt COMMA outImageSpec, /* Arguments */
	{
		outImageSpec = std::make_shared<ccl::ImageSpec>(xres,yres,nchans,fmt);
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,find_node_type, /* namespace, function name */
	const ccl::NodeType*, /* returnType */
	(const char *name), /* Parameters */
	name, /* Arguments */
	{
		return ccl::NodeType::find(ccl::ustring{name});
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,path_init, /* namespace, function name */
	void, /* returnType */
	(const char *path, const char *userPath), /* Parameters */
	path COMMA userPath, /* Arguments */
	{
		ccl::path_init(path,userPath);
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,get_colorspace_raw, /* namespace, function name */
	void, /* returnType */
	(char **outStr, size_t& outStrLen), /* Parameters */
	outStr COMMA outStrLen, /* Arguments */
	{
		icycles::detail::str_to_cstr(ccl::u_colorspace_raw.c_str(),outStr,outStrLen);
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,get_colorspace_srgb, /* namespace, function name */
	void, /* returnType */
	(char **outStr, size_t& outStrLen), /* Parameters */
	outStr COMMA outStrLen, /* Arguments */
	{
		icycles::detail::str_to_cstr(ccl::u_colorspace_srgb.c_str(),outStr,outStrLen);
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,string_iequals, /* namespace, function name */
	bool, /* returnType */
	(const char *str0, const char *str1), /* Parameters */
	str0 COMMA str1, /* Arguments */
	{
		return ccl::string_iequals(str0,str1);
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,create_buffer_params, /* namespace, function name */
	void, /* returnType */
	(icycles::BufferParams &outParams), /* Parameters */
	outParams, /* Arguments */
	{
		outParams = icycles::BufferParams{new ccl::BufferParams{} COMMA [](ccl::BufferParams *ptr) {
			icycles::detail::free(ptr);
		}};
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,create_scene_params, /* namespace, function name */
	void, /* returnType */
	(icycles::SceneParams &outParams), /* Parameters */
	outParams, /* Arguments */
	{
		outParams = icycles::SceneParams{new ccl::SceneParams{} COMMA [](ccl::SceneParams *ptr) {
			icycles::detail::free(ptr);
		}};
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,create_session_params, /* namespace, function name */
	void, /* returnType */
	(icycles::SessionParams &outParams), /* Parameters */
	outParams, /* Arguments */
	{
		outParams = icycles::SessionParams{new ccl::SessionParams{} COMMA [](ccl::SessionParams *ptr) {
			icycles::detail::free(ptr);
		}};
	}
)

#ifdef ENABLE_CYCLES_LOGGING
DEFINE_SIMPLE_FUNCTION(
	util,enable_logging, /* namespace, function name */
	void, /* returnType */
	(const char *appName, const char *infoLog, const char *warningLog, const char *errorLog, const char *fatalLog, const char *logDir), /* Parameters */
	appName COMMA infoLog COMMA warningLog COMMA errorLog COMMA fatalLog COMMA logDir, /* Arguments */
	{
		// ccl::util_logging_init("util_raytracing");
		// ccl::util_logging_verbosity_set(2);
		// ccl::util_logging_start();
		google::InitGoogleLogging(appName);
		google::SetLogDestination(google::GLOG_INFO,infoLog);
		google::SetLogDestination(google::GLOG_WARNING,warningLog);
		google::SetLogDestination(google::GLOG_ERROR,errorLog);
		google::SetLogDestination(google::GLOG_FATAL,fatalLog);
		FLAGS_log_dir = logDir;
		//FLAGS_logtostderr = true;
		//FLAGS_alsologtostderr = true; // Doesn't seem to work properly?
		//FLAGS_stderrthreshold = google::GLOG_WARNING|google::GLOG_ERROR|google::GLOG_INFO|google::GLOG_FATAL;
		//FLAGS_v = 5; // Setting the log level any other way doesn't seem to work properly
		// LOG(INFO) << "Info Test 1";
		// google::LogAtLevel(google::GLOG_INFO,"Info test");
		// google::LogAtLevel(google::GLOG_WARNING,"Warning test");
	}
)
DEFINE_SIMPLE_FUNCTION(
	util,flush_log, /* namespace, function name */
	void, /* returnType */
	(), /* Parameters */
	, /* Arguments */
	{
		google::FlushLogFiles(google::GLOG_INFO);
		google::FlushLogFiles(google::GLOG_WARNING);
		google::FlushLogFiles(google::GLOG_ERROR);
		google::FlushLogFiles(google::GLOG_FATAL);
	}
)
#endif

#endif
