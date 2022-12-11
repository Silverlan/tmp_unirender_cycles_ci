/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2020 Florian Weischer
*/

#include "util_raytracing/shader.hpp"
#include "unirender/cycles/ccl_shader.hpp"
#include "unirender/cycles/cycles_interface.hpp"
#include "util_raytracing/scene.hpp"
#include "util_raytracing/mesh.hpp"
#include "util_raytracing/exception.hpp"
#include "unirender/cycles/renderer.hpp"
#include <sharedutils/util_path.hpp>
#pragma optimize("",off)
ccl::NodeMathType unirender::cycles::to_ccl_type(unirender::nodes::math::MathType type)
{
	switch(type)
	{
		case unirender::nodes::math::MathType::Add:
			return ccl::NodeMathType::NODE_MATH_ADD;
		case unirender::nodes::math::MathType::Subtract:
			return ccl::NodeMathType::NODE_MATH_SUBTRACT;
		case unirender::nodes::math::MathType::Multiply:
			return ccl::NodeMathType::NODE_MATH_MULTIPLY;
		case unirender::nodes::math::MathType::Divide:
			return ccl::NodeMathType::NODE_MATH_DIVIDE;
		case unirender::nodes::math::MathType::Sine:
			return ccl::NodeMathType::NODE_MATH_SINE;
		case unirender::nodes::math::MathType::Cosine:
			return ccl::NodeMathType::NODE_MATH_COSINE;
		case unirender::nodes::math::MathType::Tangent:
			return ccl::NodeMathType::NODE_MATH_TANGENT;
		case unirender::nodes::math::MathType::ArcSine:
			return ccl::NodeMathType::NODE_MATH_ARCSINE;
		case unirender::nodes::math::MathType::ArcCosine:
			return ccl::NodeMathType::NODE_MATH_ARCCOSINE;
		case unirender::nodes::math::MathType::ArcTangent:
			return ccl::NodeMathType::NODE_MATH_ARCTANGENT;
		case unirender::nodes::math::MathType::Power:
			return ccl::NodeMathType::NODE_MATH_POWER;
		case unirender::nodes::math::MathType::Logarithm:
			return ccl::NodeMathType::NODE_MATH_LOGARITHM;
		case unirender::nodes::math::MathType::Minimum:
			return ccl::NodeMathType::NODE_MATH_MINIMUM;
		case unirender::nodes::math::MathType::Maximum:
			return ccl::NodeMathType::NODE_MATH_MAXIMUM;
		case unirender::nodes::math::MathType::Round:
			return ccl::NodeMathType::NODE_MATH_ROUND;
		case unirender::nodes::math::MathType::LessThan:
			return ccl::NodeMathType::NODE_MATH_LESS_THAN;
		case unirender::nodes::math::MathType::GreaterThan:
			return ccl::NodeMathType::NODE_MATH_GREATER_THAN;
		case unirender::nodes::math::MathType::Modulo:
			return ccl::NodeMathType::NODE_MATH_MODULO;
		case unirender::nodes::math::MathType::Absolute:
			return ccl::NodeMathType::NODE_MATH_ABSOLUTE;
		case unirender::nodes::math::MathType::ArcTan2:
			return ccl::NodeMathType::NODE_MATH_ARCTAN2;
		case unirender::nodes::math::MathType::Floor:
			return ccl::NodeMathType::NODE_MATH_FLOOR;
		case unirender::nodes::math::MathType::Ceil:
			return ccl::NodeMathType::NODE_MATH_CEIL;
		case unirender::nodes::math::MathType::Fraction:
			return ccl::NodeMathType::NODE_MATH_FRACTION;
		case unirender::nodes::math::MathType::Sqrt:
			return ccl::NodeMathType::NODE_MATH_SQRT;
		case unirender::nodes::math::MathType::InvSqrt:
			return ccl::NodeMathType::NODE_MATH_INV_SQRT;
		case unirender::nodes::math::MathType::Sign:
			return ccl::NodeMathType::NODE_MATH_SIGN;
		case unirender::nodes::math::MathType::Exponent:
			return ccl::NodeMathType::NODE_MATH_EXPONENT;
		case unirender::nodes::math::MathType::Radians:
			return ccl::NodeMathType::NODE_MATH_RADIANS;
		case unirender::nodes::math::MathType::Degrees:
			return ccl::NodeMathType::NODE_MATH_DEGREES;
		case unirender::nodes::math::MathType::SinH:
			return ccl::NodeMathType::NODE_MATH_SINH;
		case unirender::nodes::math::MathType::CosH:
			return ccl::NodeMathType::NODE_MATH_COSH;
		case unirender::nodes::math::MathType::TanH:
			return ccl::NodeMathType::NODE_MATH_TANH;
		case unirender::nodes::math::MathType::Trunc:
			return ccl::NodeMathType::NODE_MATH_TRUNC;
		case unirender::nodes::math::MathType::Snap:
			return ccl::NodeMathType::NODE_MATH_SNAP;
		case unirender::nodes::math::MathType::Wrap:
			return ccl::NodeMathType::NODE_MATH_WRAP;
		case unirender::nodes::math::MathType::Compare:
			return ccl::NodeMathType::NODE_MATH_COMPARE;
		case unirender::nodes::math::MathType::MultiplyAdd:
			return ccl::NodeMathType::NODE_MATH_MULTIPLY_ADD;
		case unirender::nodes::math::MathType::PingPong:
			return ccl::NodeMathType::NODE_MATH_PINGPONG;
		case unirender::nodes::math::MathType::SmoothMin:
			return ccl::NodeMathType::NODE_MATH_SMOOTH_MIN;
		case unirender::nodes::math::MathType::SmoothMax:
			return ccl::NodeMathType::NODE_MATH_SMOOTH_MAX;
	}
	static_assert(umath::to_integral(unirender::nodes::math::MathType::Add) == ccl::NodeMathType::NODE_MATH_ADD && umath::to_integral(unirender::nodes::math::MathType::SmoothMax) == ccl::NodeMathType::NODE_MATH_SMOOTH_MAX);
	static_assert(umath::to_integral(unirender::nodes::math::MathType::Count) == 40);
}
ccl::NodeVectorTransformType unirender::cycles::to_ccl_type(unirender::nodes::vector_transform::Type type)
{
	switch(type)
	{
	case unirender::nodes::vector_transform::Type::Vector:
		return ccl::NodeVectorTransformType::NODE_VECTOR_TRANSFORM_TYPE_VECTOR;
	case unirender::nodes::vector_transform::Type::Point:
		return ccl::NodeVectorTransformType::NODE_VECTOR_TRANSFORM_TYPE_POINT;
	case unirender::nodes::vector_transform::Type::Normal:
		return ccl::NodeVectorTransformType::NODE_VECTOR_TRANSFORM_TYPE_NORMAL;
	};
	static_assert(umath::to_integral(unirender::nodes::vector_transform::Type::Count) == 4);
}
ccl::NodeVectorMathType unirender::cycles::to_ccl_type(unirender::nodes::vector_math::MathType type)
{
	switch(type)
	{
	case unirender::nodes::vector_math::MathType::Add:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_ADD;
	case unirender::nodes::vector_math::MathType::Subtract:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_SUBTRACT;
	case unirender::nodes::vector_math::MathType::Multiply:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_MULTIPLY;
	case unirender::nodes::vector_math::MathType::Divide:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_DIVIDE;

	case unirender::nodes::vector_math::MathType::CrossProduct:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_CROSS_PRODUCT;
	case unirender::nodes::vector_math::MathType::Project:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_PROJECT;
	case unirender::nodes::vector_math::MathType::Reflect:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_REFLECT;
	case unirender::nodes::vector_math::MathType::DotProduct:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_DOT_PRODUCT;

	case unirender::nodes::vector_math::MathType::Distance:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_DISTANCE;
	case unirender::nodes::vector_math::MathType::Length:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_LENGTH;
	case unirender::nodes::vector_math::MathType::Scale:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_SCALE;
	case unirender::nodes::vector_math::MathType::Normalize:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_NORMALIZE;

	case unirender::nodes::vector_math::MathType::Snap:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_SNAP;
	case unirender::nodes::vector_math::MathType::Floor:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_FLOOR;
	case unirender::nodes::vector_math::MathType::Ceil:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_CEIL;
	case unirender::nodes::vector_math::MathType::Modulo:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_MODULO;
	case unirender::nodes::vector_math::MathType::Fraction:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_FRACTION;
	case unirender::nodes::vector_math::MathType::Absolute:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_ABSOLUTE;
	case unirender::nodes::vector_math::MathType::Minimum:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_MINIMUM;
	case unirender::nodes::vector_math::MathType::Maximum:
		return ccl::NodeVectorMathType::NODE_VECTOR_MATH_MAXIMUM;
	};
	static_assert(umath::to_integral(unirender::nodes::vector_math::MathType::Add) == ccl::NodeVectorMathType::NODE_VECTOR_MATH_ADD && umath::to_integral(unirender::nodes::vector_math::MathType::Maximum) == ccl::NodeVectorMathType::NODE_VECTOR_MATH_MAXIMUM);
	static_assert(umath::to_integral(unirender::nodes::vector_math::MathType::Count) == 20);
}

std::string unirender::cycles::to_ccl_type(unirender::ColorSpace space)
{
	icycles::CString cstr {};
	switch(space)
	{
	case unirender::ColorSpace::Raw:
	{
		icycles::util::get_colorspace_raw(&cstr.cstr,cstr.len);
		return cstr;
	}
	case unirender::ColorSpace::Srgb:
	{
		icycles::util::get_colorspace_srgb(&cstr.cstr,cstr.len);
		return cstr;
	}
	}
	static_assert(umath::to_integral(unirender::ColorSpace::Count) == 3);
}

ccl::NodeEnvironmentProjection unirender::cycles::to_ccl_type(unirender::EnvironmentProjection projection)
{
	switch(projection)
	{
	case unirender::EnvironmentProjection::Equirectangular:
		return ccl::NodeEnvironmentProjection::NODE_ENVIRONMENT_EQUIRECTANGULAR;
	case unirender::EnvironmentProjection::MirrorBall:
		return ccl::NodeEnvironmentProjection::NODE_ENVIRONMENT_MIRROR_BALL;
	}
	static_assert(umath::to_integral(unirender::EnvironmentProjection::Equirectangular) == ccl::NodeEnvironmentProjection::NODE_ENVIRONMENT_EQUIRECTANGULAR && umath::to_integral(unirender::EnvironmentProjection::MirrorBall) == ccl::NodeEnvironmentProjection::NODE_ENVIRONMENT_MIRROR_BALL);
	static_assert(umath::to_integral(unirender::EnvironmentProjection::Count) == 2);
}

ccl::ClosureType unirender::cycles::to_ccl_type(unirender::ClosureType type)
{
	switch(type)
	{
	case unirender::ClosureType::BsdfMicroFacetMultiGgxGlass:
		return ccl::ClosureType::CLOSURE_BSDF_MICROFACET_MULTI_GGX_GLASS_ID;
	case unirender::ClosureType::BsdfDiffuseToon:
		return ccl::ClosureType::CLOSURE_BSDF_DIFFUSE_TOON_ID;
	case unirender::ClosureType::BsdfMicroFacetGgxGlass:
		return ccl::ClosureType::CLOSURE_BSDF_MICROFACET_GGX_GLASS_ID;
	}
	static_assert(umath::to_integral(unirender::EnvironmentProjection::Equirectangular) == ccl::NodeEnvironmentProjection::NODE_ENVIRONMENT_EQUIRECTANGULAR && umath::to_integral(unirender::EnvironmentProjection::MirrorBall) == ccl::NodeEnvironmentProjection::NODE_ENVIRONMENT_MIRROR_BALL);
	static_assert(umath::to_integral(unirender::ClosureType::Count) == 4);
}

ccl::ImageAlphaType unirender::cycles::to_ccl_type(unirender::nodes::image_texture::AlphaType type)
{
	switch(type)
	{
	case unirender::nodes::image_texture::AlphaType::Unassociated:
		return ccl::ImageAlphaType::IMAGE_ALPHA_UNASSOCIATED;
	case unirender::nodes::image_texture::AlphaType::Associated:
		return ccl::ImageAlphaType::IMAGE_ALPHA_ASSOCIATED;
	case unirender::nodes::image_texture::AlphaType::ChannelPacked:
		return ccl::ImageAlphaType::IMAGE_ALPHA_CHANNEL_PACKED;
	case unirender::nodes::image_texture::AlphaType::Ignore:
		return ccl::ImageAlphaType::IMAGE_ALPHA_IGNORE;
	case unirender::nodes::image_texture::AlphaType::Auto:
		return ccl::ImageAlphaType::IMAGE_ALPHA_AUTO;
	}
	static_assert(umath::to_integral(unirender::nodes::image_texture::AlphaType::Unassociated) == ccl::ImageAlphaType::IMAGE_ALPHA_UNASSOCIATED && umath::to_integral(unirender::nodes::image_texture::AlphaType::Auto) == ccl::ImageAlphaType::IMAGE_ALPHA_AUTO);
	static_assert(umath::to_integral(unirender::nodes::image_texture::AlphaType::Count) == 5);
}

ccl::InterpolationType unirender::cycles::to_ccl_type(unirender::nodes::image_texture::InterpolationType type)
{
	switch(type)
	{
	case unirender::nodes::image_texture::InterpolationType::Linear:
		return ccl::InterpolationType::INTERPOLATION_LINEAR;
	case unirender::nodes::image_texture::InterpolationType::Closest:
		return ccl::InterpolationType::INTERPOLATION_CLOSEST;
	case unirender::nodes::image_texture::InterpolationType::Cubic:
		return ccl::InterpolationType::INTERPOLATION_CUBIC;
	case unirender::nodes::image_texture::InterpolationType::Smart:
		return ccl::InterpolationType::INTERPOLATION_SMART;
	}
	static_assert(umath::to_integral(unirender::nodes::image_texture::InterpolationType::Linear) == ccl::InterpolationType::INTERPOLATION_LINEAR && umath::to_integral(unirender::nodes::image_texture::InterpolationType::Smart) == ccl::InterpolationType::INTERPOLATION_SMART);
	static_assert(umath::to_integral(unirender::nodes::image_texture::InterpolationType::Count) == 4);
}

ccl::ExtensionType unirender::cycles::to_ccl_type(unirender::nodes::image_texture::ExtensionType type)
{
	switch(type)
	{
	case unirender::nodes::image_texture::ExtensionType::Repeat:
		return ccl::ExtensionType::EXTENSION_REPEAT;
	case unirender::nodes::image_texture::ExtensionType::Extend:
		return ccl::ExtensionType::EXTENSION_EXTEND;
	case unirender::nodes::image_texture::ExtensionType::Clip:
		return ccl::ExtensionType::EXTENSION_CLIP;
	}
	static_assert(umath::to_integral(unirender::nodes::image_texture::ExtensionType::Repeat) == ccl::ExtensionType::EXTENSION_REPEAT && umath::to_integral(unirender::nodes::image_texture::ExtensionType::Clip) == ccl::ExtensionType::EXTENSION_CLIP);
	static_assert(umath::to_integral(unirender::nodes::image_texture::ExtensionType::Count) == 3);
}

ccl::NodeImageProjection unirender::cycles::to_ccl_type(unirender::nodes::image_texture::Projection type)
{
	switch(type)
	{
	case unirender::nodes::image_texture::Projection::Flat:
		return ccl::NodeImageProjection::NODE_IMAGE_PROJ_FLAT;
	case unirender::nodes::image_texture::Projection::Box:
		return ccl::NodeImageProjection::NODE_IMAGE_PROJ_BOX;
	case unirender::nodes::image_texture::Projection::Sphere:
		return ccl::NodeImageProjection::NODE_IMAGE_PROJ_SPHERE;
	case unirender::nodes::image_texture::Projection::Tube:
		return ccl::NodeImageProjection::NODE_IMAGE_PROJ_TUBE;
	}
	static_assert(umath::to_integral(unirender::nodes::image_texture::Projection::Flat) == ccl::NodeImageProjection::NODE_IMAGE_PROJ_FLAT && umath::to_integral(unirender::nodes::image_texture::Projection::Tube) == ccl::NodeImageProjection::NODE_IMAGE_PROJ_TUBE);
	static_assert(umath::to_integral(unirender::nodes::image_texture::Projection::Count) == 4);
}

ccl::NodeMappingType unirender::cycles::to_ccl_type(unirender::nodes::mapping::Type type)
{
	switch(type)
	{
	case unirender::nodes::mapping::Type::Point:
		return ccl::NodeMappingType::NODE_MAPPING_TYPE_POINT;
	case unirender::nodes::mapping::Type::Texture:
		return ccl::NodeMappingType::NODE_MAPPING_TYPE_TEXTURE;
	case unirender::nodes::mapping::Type::Vector:
		return ccl::NodeMappingType::NODE_MAPPING_TYPE_VECTOR;
	case unirender::nodes::mapping::Type::Normal:
		return ccl::NodeMappingType::NODE_MAPPING_TYPE_NORMAL;
	}
	static_assert(umath::to_integral(unirender::nodes::mapping::Type::Point) == ccl::NodeMappingType::NODE_MAPPING_TYPE_POINT && umath::to_integral(unirender::nodes::mapping::Type::Normal) == ccl::NodeMappingType::NODE_MAPPING_TYPE_NORMAL);
	static_assert(umath::to_integral(unirender::nodes::mapping::Type::Count) == 4);
}

ccl::NodeNormalMapSpace unirender::cycles::to_ccl_type(unirender::nodes::normal_map::Space space)
{
	switch(space)
	{
	case unirender::nodes::normal_map::Space::Tangent:
		return ccl::NodeNormalMapSpace::NODE_NORMAL_MAP_TANGENT;
	case unirender::nodes::normal_map::Space::Object:
		return ccl::NodeNormalMapSpace::NODE_NORMAL_MAP_OBJECT;
	case unirender::nodes::normal_map::Space::World:
		return ccl::NodeNormalMapSpace::NODE_NORMAL_MAP_WORLD;
	}
	static_assert(umath::to_integral(unirender::nodes::normal_map::Space::Tangent) == ccl::NodeNormalMapSpace::NODE_NORMAL_MAP_TANGENT && umath::to_integral(unirender::nodes::normal_map::Space::World) == ccl::NodeNormalMapSpace::NODE_NORMAL_MAP_WORLD);
	static_assert(umath::to_integral(unirender::nodes::normal_map::Space::Count) == 3);
}

ccl::NodeMix unirender::cycles::to_ccl_type(unirender::nodes::mix::Mix mix)
{
	switch(mix)
	{
	case unirender::nodes::mix::Mix::Blend:
		return ccl::NodeMix::NODE_MIX_BLEND;
	case unirender::nodes::mix::Mix::Add:
		return ccl::NodeMix::NODE_MIX_ADD;
	case unirender::nodes::mix::Mix::Mul:
		return ccl::NodeMix::NODE_MIX_MUL;
	case unirender::nodes::mix::Mix::Sub:
		return ccl::NodeMix::NODE_MIX_SUB;
	case unirender::nodes::mix::Mix::Screen:
		return ccl::NodeMix::NODE_MIX_SCREEN;
	case unirender::nodes::mix::Mix::Div:
		return ccl::NodeMix::NODE_MIX_DIV;
	case unirender::nodes::mix::Mix::Diff:
		return ccl::NodeMix::NODE_MIX_DIFF;
	case unirender::nodes::mix::Mix::Dark:
		return ccl::NodeMix::NODE_MIX_DARK;
	case unirender::nodes::mix::Mix::Light:
		return ccl::NodeMix::NODE_MIX_LIGHT;
	case unirender::nodes::mix::Mix::Overlay:
		return ccl::NodeMix::NODE_MIX_OVERLAY;
	case unirender::nodes::mix::Mix::Dodge:
		return ccl::NodeMix::NODE_MIX_DODGE;
	case unirender::nodes::mix::Mix::Burn:
		return ccl::NodeMix::NODE_MIX_BURN;
	case unirender::nodes::mix::Mix::Hue:
		return ccl::NodeMix::NODE_MIX_HUE;
	case unirender::nodes::mix::Mix::Sat:
		return ccl::NodeMix::NODE_MIX_SAT;
	case unirender::nodes::mix::Mix::Val:
		return ccl::NodeMix::NODE_MIX_VAL;
	case unirender::nodes::mix::Mix::Color:
		return ccl::NodeMix::NODE_MIX_COLOR;
	case unirender::nodes::mix::Mix::Soft:
		return ccl::NodeMix::NODE_MIX_SOFT;
	case unirender::nodes::mix::Mix::Linear:
		return ccl::NodeMix::NODE_MIX_LINEAR;
	case unirender::nodes::mix::Mix::Clamp:
		return ccl::NodeMix::NODE_MIX_CLAMP;
	}
	static_assert(umath::to_integral(unirender::nodes::mix::Mix::Blend) == ccl::NodeMix::NODE_MIX_BLEND && umath::to_integral(unirender::nodes::mix::Mix::Clamp) == ccl::NodeMix::NODE_MIX_CLAMP);
	static_assert(umath::to_integral(unirender::nodes::mix::Mix::Count) == 19);
}
ccl::NodeVectorTransformConvertSpace unirender::cycles::to_ccl_type(unirender::nodes::vector_transform::ConvertSpace convertSpace)
{
	switch(convertSpace)
	{
	case unirender::nodes::vector_transform::ConvertSpace::World:
		return ccl::NodeVectorTransformConvertSpace::NODE_VECTOR_TRANSFORM_CONVERT_SPACE_WORLD;
	case unirender::nodes::vector_transform::ConvertSpace::Object:
		return ccl::NodeVectorTransformConvertSpace::NODE_VECTOR_TRANSFORM_CONVERT_SPACE_OBJECT;
	case unirender::nodes::vector_transform::ConvertSpace::Camera:
		return ccl::NodeVectorTransformConvertSpace::NODE_VECTOR_TRANSFORM_CONVERT_SPACE_CAMERA;
	}
	static_assert(umath::to_integral(unirender::nodes::vector_transform::ConvertSpace::World) == ccl::NodeVectorTransformConvertSpace::NODE_VECTOR_TRANSFORM_CONVERT_SPACE_WORLD && umath::to_integral(unirender::nodes::vector_transform::ConvertSpace::Camera) == ccl::NodeVectorTransformConvertSpace::NODE_VECTOR_TRANSFORM_CONVERT_SPACE_CAMERA);
	static_assert(umath::to_integral(unirender::nodes::vector_transform::ConvertSpace::Count) == 3);
}
std::shared_ptr<unirender::CCLShader> unirender::CCLShader::Create(cycles::Renderer &renderer,ccl::Shader &cclShader,const GroupNodeDesc &desc,bool useCache)
{
	if(useCache)
	{
		auto shader = renderer.GetCachedShader(desc);
		if(shader)
			return shader;
	}
	icycles::shader::set_volume_sampling_method(cclShader,ccl::VOLUME_SAMPLING_MULTIPLE_IMPORTANCE);

	auto *graph = icycles::shader_graph::create();
	auto pShader = std::shared_ptr<CCLShader>{new CCLShader{renderer,cclShader,*graph}};
	pShader->m_flags |= Flags::CCLShaderOwnedByScene;

	pShader->InitializeNodeGraph(desc);
	if(useCache)
		renderer.AddShader(*pShader,&desc);
	return pShader;
}
std::shared_ptr<unirender::CCLShader> unirender::CCLShader::Create(cycles::Renderer &renderer,const GroupNodeDesc &desc)
{
	auto shader = renderer.GetCachedShader(desc);
	if(shader)
		return shader;
	auto *cclShader = icycles::shader::create();
	icycles::node::set_name(*cclShader,desc.GetName().c_str());
	renderer.GetCclScene()->shaders.push_back(cclShader);
	shader = Create(renderer,*cclShader,desc,true);
	auto apiData = renderer.GetApiData();

	auto dontSimplify = false;
	apiData.GetFromPath("cycles/shader/dontSimplifyGraphs")(dontSimplify);
	if(!dontSimplify)
		icycles::shader_graph::simplify(shader->m_cclGraph,renderer.GetCclScene());

	auto dumpGraphs = false;
	apiData.GetFromPath("cycles/debug/dump_shader_graphs")(dumpGraphs);
	if(dumpGraphs)
	{
		auto &graph = shader->m_cclGraph;
		auto localDumpPath = util::Path::CreatePath("temp/cycles/graph_dump");
		filemanager::create_path(localDumpPath.GetString());
		auto dumpPath = util::Path::CreatePath(util::get_program_path()) +localDumpPath;
		auto idx = desc.GetIndex();
		std::string fileName = "graph_" +util::uuid_to_string(util::generate_uuid_v4()) +".txt";
		auto filePath = dumpPath +util::Path::CreateFile(fileName);
		icycles::shader_graph::dump_graph(graph,filePath.GetString().c_str());
	}
	return shader;
}

unirender::CCLShader::CCLShader(cycles::Renderer &renderer,ccl::Shader &cclShader,ccl::ShaderGraph &cclShaderGraph)
	: m_renderer{renderer},m_cclShader{cclShader},m_cclGraph{cclShaderGraph}
{}

unirender::CCLShader::~CCLShader()
{
	if(umath::is_flag_set(m_flags,Flags::CCLShaderGraphOwnedByScene) == false)
		icycles::shader_graph::destroy(&m_cclGraph);
	if(umath::is_flag_set(m_flags,Flags::CCLShaderOwnedByScene) == false)
		icycles::shader::destroy(&m_cclShader);
}

ccl::Shader *unirender::CCLShader::operator->() {return &m_cclShader;}
ccl::Shader *unirender::CCLShader::operator*() {return &m_cclShader;}

void unirender::CCLShader::DoFinalize(Scene &scene)
{
	BaseObject::DoFinalize(scene);
	m_flags |= Flags::CCLShaderGraphOwnedByScene | Flags::CCLShaderOwnedByScene;
	icycles::shader::set_graph(m_cclShader,&m_cclGraph);
	icycles::shader::tag_update(m_cclShader,m_renderer.GetCclScene());
}

std::unique_ptr<unirender::CCLShader::BaseNodeWrapper> unirender::CCLShader::ResolveCustomNode(const std::string &typeName)
{
	if(typeName == unirender::NODE_NORMAL_TEXTURE)
	{
		struct NormalNodeWrapper : public BaseNodeWrapper
		{
			virtual ccl::ShaderInput *FindInput(const std::string &name,ccl::ShaderNode **outNode) override
			{
				if(name == unirender::nodes::normal_texture::IN_STRENGTH)
				{
					*outNode = normalMapNode;
					return unirender::CCLShader::FindInput(*normalMapNode,unirender::nodes::normal_map::IN_STRENGTH);
				}
				return nullptr;
			}
			virtual ccl::ShaderOutput *FindOutput(const std::string &name,ccl::ShaderNode **outNode) override
			{
				if(name == unirender::nodes::normal_texture::OUT_NORMAL)
				{
					*outNode = normalMapNode;
					return unirender::CCLShader::FindOutput(*normalMapNode,unirender::nodes::normal_map::OUT_NORMAL);
				}
				return nullptr;
			}
			virtual const ccl::SocketType *FindProperty(const std::string &name,ccl::ShaderNode **outNode) override
			{
				if(name == unirender::nodes::normal_texture::IN_FILENAME)
				{
					*outNode = imageTexNode;
					return icycles::node_type::find_input(*icycles::node::get_type(*imageTexNode),unirender::nodes::image_texture::IN_FILENAME);
				}
				return nullptr;
			}
			virtual ccl::ShaderNode *GetOutputNode() override {return normalMapNode;}
			ccl::ImageTextureNode* imageTexNode = nullptr;
			ccl::NormalMapNode *normalMapNode = nullptr;
		};
		auto wrapper = std::make_unique<NormalNodeWrapper>();
		wrapper->imageTexNode = static_cast<ccl::ImageTextureNode*>(AddNode(unirender::NODE_IMAGE_TEXTURE));
		assert(wrapper->imageTexNode);
		icycles::CString cstr {};
		icycles::util::get_colorspace_raw(&cstr.cstr,cstr.len);
		icycles::image_texture_node::set_colorspace(*wrapper->imageTexNode,cstr.cstr);

		auto *sep = static_cast<ccl::SeparateRGBNode*>(AddNode(unirender::NODE_SEPARATE_RGB));
		icycles::shader_graph::connect(m_cclGraph,FindOutput(*wrapper->imageTexNode,unirender::nodes::image_texture::OUT_COLOR),FindInput(*sep,unirender::nodes::separate_rgb::IN_COLOR));

		auto *cmb = static_cast<ccl::CombineRGBNode*>(AddNode(unirender::NODE_COMBINE_RGB));
		icycles::shader_graph::connect(m_cclGraph,FindOutput(*sep,unirender::nodes::separate_rgb::OUT_R),FindInput(*cmb,unirender::nodes::combine_rgb::IN_G));
		icycles::shader_graph::connect(m_cclGraph,FindOutput(*sep,unirender::nodes::separate_rgb::OUT_G),FindInput(*cmb,unirender::nodes::combine_rgb::IN_R));
		icycles::shader_graph::connect(m_cclGraph,FindOutput(*sep,unirender::nodes::separate_rgb::OUT_B),FindInput(*cmb,unirender::nodes::combine_rgb::IN_B));
		
		wrapper->normalMapNode = static_cast<ccl::NormalMapNode*>(AddNode(unirender::NODE_NORMAL_MAP));
		assert(wrapper->normalMapNode);
		icycles::normal_map_node::set_space(*wrapper->normalMapNode,ccl::NodeNormalMapSpace::NODE_NORMAL_MAP_TANGENT);

		auto *normIn = FindInput(*wrapper->normalMapNode,unirender::nodes::normal_map::IN_COLOR);
		icycles::shader_graph::connect(m_cclGraph,FindOutput(*cmb,unirender::nodes::combine_rgb::OUT_IMAGE),normIn);
		return wrapper;
	}
	return nullptr;
}

ccl::ShaderNode *unirender::CCLShader::AddNode(const std::string &typeName)
{
	auto *nodeType = icycles::util::find_node_type(typeName.c_str());
	auto *snode = nodeType ? static_cast<ccl::ShaderNode*>(icycles::node_type::create_node(*nodeType)) : nullptr;
	if(snode == nullptr)
		return nullptr;

	auto name = GetCurrentInternalNodeName();
	icycles::shader_node::set_owner(*snode,&m_cclGraph);
	icycles::node::set_name(*snode,name.c_str());
	icycles::shader_graph::add(m_cclGraph,snode);
	return snode;
}

void unirender::CCLShader::InitializeNode(const NodeDesc &desc,std::unordered_map<const NodeDesc*,ccl::ShaderNode*> &nodeToCclNode,const GroupSocketTranslationTable &groupIoSockets)
{
	if(desc.IsGroupNode())
	{
		auto &groupDesc = *static_cast<const GroupNodeDesc*>(&desc);
		auto &childNodes = groupDesc.GetChildNodes();
		for(auto &childNode : childNodes)
			InitializeNode(*childNode,nodeToCclNode,groupIoSockets);
		
		auto getCclSocket = [this,&groupIoSockets,&nodeToCclNode](const Socket &socket,bool input) -> std::optional<std::pair<ccl::ShaderNode*,std::string>> {
			auto it = groupIoSockets.find(socket);
			if(it != groupIoSockets.end())
				return input ? it->second.input : it->second.output;
			std::string socketName;
			auto *node = socket.GetNode(socketName);
			auto itNode = nodeToCclNode.find(node);
			if(itNode == nodeToCclNode.end())
			{
				// m_scene.HandleError("Unable to locate ccl node for from node '" +node->GetName() +"'!");
				return {};
			}
			auto *cclNode = itNode->second;
			return std::pair<ccl::ShaderNode*,std::string>{cclNode,socketName};
		};
		auto &links = groupDesc.GetLinks();
		for(auto &link : links)
		{
			auto cclFromSocket = getCclSocket(link.fromSocket,false);
			auto cclToSocket = getCclSocket(link.toSocket,true);
			if(cclFromSocket.has_value() == false || cclToSocket.has_value() == false)
			{
				if(cclToSocket.has_value() && link.fromSocket.IsNodeSocket() && link.fromSocket.IsOutputSocket() == false)
				{
					std::string fromSocketName;
					auto *fromNode = link.fromSocket.GetNode(fromSocketName);
					if(fromNode)
					{
						auto *fromSocketDesc = fromNode->FindPropertyDesc(fromSocketName);
						// This is a special case where the input socket is actually a property,
						// so instead of linking, we just assign the property value directly.
						auto inputName = TranslateInputName(*cclToSocket->first,cclToSocket->second);
						auto *prop = FindProperty(*cclToSocket->first,inputName);
						if(fromSocketDesc && fromSocketDesc->dataValue.value)
						{
							if(prop)
								ApplySocketValue(*cclToSocket->first,inputName,*fromSocketDesc,*cclToSocket->first,*prop);
							else
								m_renderer.GetScene().HandleError("Attempted to use unknown property '" +inputName +"' with node of type '" +std::string{typeid(*cclToSocket->first).name()} +"'!");
						}
							
					}
				}
				continue;
			}
			auto *output = FindOutput(*cclFromSocket->first,cclFromSocket->second);
			auto *input = FindInput(*cclToSocket->first,cclToSocket->second);
			if(output == nullptr)
			{
				m_renderer.GetScene().HandleError("Invalid CCL output '" +cclFromSocket->second +"' for node of type '" +std::string{typeid(*cclFromSocket->first).name()} +"'!");
				continue;
			}
			if(input == nullptr)
			{
				m_renderer.GetScene().HandleError("Invalid CCL input '" +cclToSocket->second +"' for node of type '" +std::string{typeid(*cclToSocket->first).name()} +"'!");
				continue;
			}
			icycles::shader_graph::connect(m_cclGraph,output,input);
		}
		return;
	}
	auto &typeName = desc.GetTypeName();
	if(typeName == "output")
	{
		// Output node already exists by default
		nodeToCclNode[&desc] = icycles::shader_graph::output(m_cclGraph);
		return;
	}
	struct CclNodeWrapper : public unirender::CCLShader::BaseNodeWrapper
	{
		virtual ccl::ShaderInput *FindInput(const std::string &name,ccl::ShaderNode **outNode) override
		{
			*outNode = node;
			return unirender::CCLShader::FindInput(*node,name);
		}
		virtual ccl::ShaderOutput *FindOutput(const std::string &name,ccl::ShaderNode **outNode) override
		{
			*outNode = node;
			return unirender::CCLShader::FindOutput(*node,name);
		}
		virtual const ccl::SocketType *FindProperty(const std::string &name,ccl::ShaderNode **outNode) override
		{
			*outNode = node;
			return icycles::node_type::find_input(*icycles::node::get_type(*node),name.c_str());
		}
		virtual ccl::ShaderNode *GetOutputNode() override {return node;}
		ccl::ShaderNode *node = nullptr;
	};

	auto *snode = AddNode(typeName);
	std::unique_ptr<unirender::CCLShader::BaseNodeWrapper> wrapper = nullptr;
	auto isCclNode = false;
	if(snode != nullptr)
	{
		wrapper = std::make_unique<CclNodeWrapper>();
		static_cast<CclNodeWrapper*>(wrapper.get())->node = snode;
		isCclNode = true;
	}
	else
	{
		auto customNode = ResolveCustomNode(typeName);
		if(customNode == nullptr)
		{
			m_renderer.GetScene().HandleError("Unable to create ccl node of type '" +typeName +"': Invalid type?");
			return;
		}
		wrapper = std::move(customNode);
	}
	for(auto &pair : desc.GetInputs())
	{
		ccl::ShaderNode *node;
		auto inputName = pair.first;
		if(isCclNode)
			inputName = TranslateInputName(*static_cast<CclNodeWrapper*>(wrapper.get())->node,inputName);
		auto *input = wrapper->FindInput(inputName,&node);
		if(input == nullptr)
		{
			m_renderer.GetScene().HandleError("Attempted to use unknown input '" +pair.first +"' with node of type '" +typeName +"'!");
			continue;
		}
		ApplySocketValue(*node,inputName,pair.second,*node,input->socket_type);
	}

	for(auto &pair : desc.GetProperties())
	{
		ccl::ShaderNode *node;
		auto inputName = pair.first;
		if(isCclNode)
			inputName = TranslateInputName(*static_cast<CclNodeWrapper*>(wrapper.get())->node,inputName);
		auto *type = wrapper->FindProperty(inputName,&node);
		if(type == nullptr)
		{
			m_renderer.GetScene().HandleError("Attempted to use unknown property '" +pair.first +"' with node of type '" +typeName +"'!");
			continue;
		}
		ApplySocketValue(*node,inputName,pair.second,*node,*type);
	}

	nodeToCclNode[&desc] = wrapper->GetOutputNode();
}

template<typename TSrc,typename TDst>
	static ccl::array<TDst> to_ccl_array(const std::vector<TSrc> &input,const std::function<TDst(const TSrc&)> &converter)
{
	ccl::array<TDst> output {};
	output.resize(input.size());
	for(auto i=decltype(input.size()){0u};i<input.size();++i)
		output[i] = converter(input.at(i));
	return output;
}

std::string unirender::CCLShader::TranslateInputName(const ccl::ShaderNode &node,const std::string &inputName)
{
	// Some Cycles node socket names don't match ours (due to Cycles updates or other reasons), so we'll have to translate them here
	auto type = icycles::shader_node::get_type_enum(node);
	switch(type)
	{
	case icycles::ShaderNodeType::Math:
	{
		if(ustring::compare(inputName.c_str(),unirender::nodes::math::IN_TYPE,false))
			return "math_type";
		break;
	}
	case icycles::ShaderNodeType::Mapping:
	{
		if(ustring::compare(inputName.c_str(),unirender::nodes::mapping::IN_TYPE,false))
			return "mapping_type";
		break;
	}
	case icycles::ShaderNodeType::Mix:
	{
		if(ustring::compare(inputName.c_str(),unirender::nodes::mix::IN_TYPE,false))
			return "mix_type";
		break;
	}
	case icycles::ShaderNodeType::VectorMath:
	{
		if(ustring::compare(inputName.c_str(),unirender::nodes::vector_math::IN_TYPE,false))
			return "math_type";
		break;
	}
	case icycles::ShaderNodeType::VectorTransform:
	{
		if(ustring::compare(inputName.c_str(),unirender::nodes::vector_transform::IN_TYPE,false))
			return "transform_type";
		break;
	}
	}
	return inputName;
}

template<icycles::ShaderNodeType nodeType,typename TEnum>
	static bool apply_translated_socket_value(
		const ccl::ShaderNode &shaderNode,const std::string &socketName,const std::string &targetSocketName,
		const unirender::NodeSocketDesc &sockDesc,ccl::Node &node,const ccl::SocketType &sockType
	)
{
	if(icycles::shader_node::get_type_enum(shaderNode) != nodeType)
		return false;
	if(!ustring::compare(socketName.c_str(),targetSocketName.c_str(),false))
		return false;
	if(sockDesc.dataValue.type != unirender::SocketType::Enum)
		return false;
	auto val = *static_cast<unirender::STEnum*>(sockDesc.dataValue.value.get());
	val = unirender::cycles::to_ccl_enum<TEnum>(val);
	icycles::node::set_int(node,sockType,val);
	return true;
}

void unirender::CCLShader::ApplySocketValue(const ccl::ShaderNode &shaderNode,const std::string &socketName,const NodeSocketDesc &sockDesc,ccl::Node &node,const ccl::SocketType &sockType)
{
	if(apply_translated_socket_value<icycles::ShaderNodeType::Math,unirender::nodes::math::MathType>(shaderNode,socketName,"math_type",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::Mapping,unirender::nodes::mapping::Type>(shaderNode,socketName,"mapping_type",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::Mix,unirender::nodes::mix::Mix>(shaderNode,socketName,"mix_type",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::VectorMath,unirender::nodes::vector_math::MathType>(shaderNode,socketName,"math_type",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::VectorTransform,unirender::nodes::vector_transform::Type>(shaderNode,socketName,"transform_type",sockDesc,node,sockType))
		return;

	if(apply_translated_socket_value<icycles::ShaderNodeType::ImageTexture,unirender::nodes::image_texture::InterpolationType>(shaderNode,socketName,"interpolation",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::ImageTexture,unirender::nodes::image_texture::ExtensionType>(shaderNode,socketName,"extension",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::ImageTexture,unirender::nodes::image_texture::Projection>(shaderNode,socketName,"projection",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::ImageTexture,unirender::nodes::image_texture::AlphaType>(shaderNode,socketName,"alpha_type",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::VectorTransform,unirender::nodes::vector_transform::ConvertSpace>(shaderNode,socketName,"convert_from",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::VectorTransform,unirender::nodes::vector_transform::ConvertSpace>(shaderNode,socketName,"convert_to",sockDesc,node,sockType))
		return;
	if(apply_translated_socket_value<icycles::ShaderNodeType::NormalMap,unirender::nodes::normal_map::Space>(shaderNode,socketName,"space",sockDesc,node,sockType))
		return;

	switch(sockDesc.dataValue.type)
	{
	case SocketType::Bool:
		static_assert(std::is_same_v<STBool,bool>);
		icycles::node::set_boolean(node,sockType,*static_cast<STBool*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Float:
		static_assert(std::is_same_v<STFloat,float>);
		icycles::node::set_float(node,sockType,*static_cast<STFloat*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Int:
		static_assert(std::is_same_v<STInt,int32_t>);
		icycles::node::set_int(node,sockType,*static_cast<STInt*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Enum:
		static_assert(std::is_same_v<STEnum,int32_t>);
		icycles::node::set_int(node,sockType,*static_cast<STEnum*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::UInt:
		static_assert(std::is_same_v<STUInt,ccl::uint>);
		icycles::node::set_uint(node,sockType,*static_cast<STUInt*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Color:
	case SocketType::Vector:
	case SocketType::Point:
	case SocketType::Normal:
	{
		static_assert(std::is_same_v<STColor,Vector3> && std::is_same_v<STVector,Vector3> && std::is_same_v<STPoint,Vector3> && std::is_same_v<STNormal,Vector3>);
		auto &v = *static_cast<STVector*>(sockDesc.dataValue.value.get());
		icycles::node::set_float3(node,sockType,ccl::float3{v.x,v.y,v.z});
		break;
	}
	case SocketType::Point2:
	{
		static_assert(std::is_same_v<STPoint2,Vector2>);
		auto &v = *static_cast<STPoint2*>(sockDesc.dataValue.value.get());
		icycles::node::set_float2(node,sockType,ccl::float2{v.x,v.y});
		break;
	}
	case SocketType::String:
	{
		static_assert(std::is_same_v<STString,std::string>);
		auto &v = *static_cast<std::string*>(sockDesc.dataValue.value.get());
		icycles::node::set_cstring(node,sockType,v.c_str());
		break;
	}
	case SocketType::Transform:
	{
		static_assert(std::is_same_v<STTransform,Mat4x3>);
		auto &v = *static_cast<Mat4x3*>(sockDesc.dataValue.value.get());
		icycles::node::set_transform(node,sockType,ccl::Transform{
			v[0][0],v[0][1],v[0][2],
			v[1][0],v[1][1],v[1][2],
			v[2][0],v[2][1],v[2][2],
			v[3][0],v[3][1],v[3][2]
		});
		break;
	}
	case SocketType::FloatArray:
	{
		static_assert(std::is_same_v<STFloatArray,std::vector<STFloat>>);
		auto &v = *static_cast<std::vector<STFloat>*>(sockDesc.dataValue.value.get());
		auto cclArray = to_ccl_array<float,float>(v,[](const float &v) -> float {return v;});
		icycles::node::set_float_a(node,sockType,cclArray);
		break;
	}
	case SocketType::ColorArray:
	{
		static_assert(std::is_same_v<STColorArray,std::vector<STColor>>);
		auto &v = *static_cast<std::vector<STColor>*>(sockDesc.dataValue.value.get());
		auto cclArray = to_ccl_array<Vector3,ccl::float3>(v,[](const Vector3 &v) -> ccl::float3 {return ccl::float3{v.x,v.y,v.z};});
		icycles::node::set_float3_a(node,sockType,cclArray);
		break;
	}
	}
	static_assert(umath::to_integral(SocketType::Count) == 16);
}

void unirender::CCLShader::ConvertGroupSocketsToNodes(const GroupNodeDesc &groupDesc,GroupSocketTranslationTable &outGroupIoSockets)
{
	// Note: Group nodes don't exist in Cycles, they're implicit and replaced by their contents.
	// To do so, we convert the input and output sockets to constant nodes and re-direct all links
	// that point to these sockets to the new nodes instead.
	auto convertGroupSocketsToNodes = [this,&groupDesc,&outGroupIoSockets](const std::unordered_map<std::string,NodeSocketDesc> &sockets,bool output) {
		for(auto &pair : sockets)
		{
			Socket socket {const_cast<GroupNodeDesc&>(groupDesc),pair.first,output};
			auto &socketDesc = pair.second;
			GroupSocketTranslation socketTranslation {};
			if(is_convertible_to(socketDesc.dataValue.type,SocketType::Float))
			{
				auto *nodeMath = static_cast<ccl::MathNode*>(AddNode(NODE_MATH));
				assert(nodeMath);
				icycles::math_node::set_math_type(*nodeMath,ccl::NodeMathType::NODE_MATH_ADD);
				icycles::math_node::set_value1(*nodeMath,0.f);
				icycles::math_node::set_value2(*nodeMath,0.f);

				if(socketDesc.dataValue.value)
				{
					auto v = socketDesc.dataValue.ToValue<float>();
					if(v.has_value())
						icycles::math_node::set_value1(*nodeMath,*v);
				}
				socketTranslation.input = {nodeMath,nodes::math::IN_VALUE1};
				socketTranslation.output = {nodeMath,nodes::math::OUT_VALUE};
			}
			else if(is_convertible_to(socketDesc.dataValue.type,SocketType::Vector))
			{
				auto *nodeVec = static_cast<ccl::VectorMathNode*>(AddNode(NODE_VECTOR_MATH));
				assert(nodeVec);
				icycles::vector_math_node::set_math_type(*nodeVec,ccl::NodeVectorMathType::NODE_VECTOR_MATH_ADD);
				icycles::math_node::set_vector1(*nodeVec,{0.f,0.f,0.f});
				icycles::math_node::set_vector2(*nodeVec,{0.f,0.f,0.f});

				if(socketDesc.dataValue.value)
				{
					auto v = socketDesc.dataValue.ToValue<Vector3>();
					if(v.has_value())
						icycles::math_node::set_vector1(*nodeVec,{v->x,v->y,v->z});
				}
				socketTranslation.input = {nodeVec,nodes::vector_math::IN_VECTOR1};
				socketTranslation.output = {nodeVec,nodes::vector_math::OUT_VECTOR};
			}
			else if(socketDesc.dataValue.type == unirender::SocketType::Closure)
			{
				auto *mix = static_cast<ccl::MixClosureNode*>(AddNode(NODE_MIX_CLOSURE));
				assert(mix);
				icycles::mix_closure_node::set_fac(*mix,0.f);

				socketTranslation.input = {mix,nodes::mix_closure::IN_CLOSURE1};
				socketTranslation.output = {mix,nodes::mix_closure::OUT_CLOSURE};
			}
			else
			{
				// m_scene.HandleError("Group node has socket of type '" +to_string(socketDesc.dataValue.type) +"', but only float and vector types are allowed!");
				continue;
			}
			outGroupIoSockets[socket] = socketTranslation;
		}
	};
	convertGroupSocketsToNodes(groupDesc.GetInputs(),false);
	convertGroupSocketsToNodes(groupDesc.GetProperties(),false);
	convertGroupSocketsToNodes(groupDesc.GetOutputs(),true);

	for(auto &node : groupDesc.GetChildNodes())
	{
		if(node->IsGroupNode() == false)
			continue;
		ConvertGroupSocketsToNodes(static_cast<GroupNodeDesc&>(*node),outGroupIoSockets);
	}
}

void unirender::CCLShader::InitializeNodeGraph(const GroupNodeDesc &desc)
{
	GroupSocketTranslationTable groupIoSockets;
	ConvertGroupSocketsToNodes(desc,groupIoSockets);

	std::unordered_map<const NodeDesc*,ccl::ShaderNode*> nodeToCclNode;
	InitializeNode(desc,nodeToCclNode,groupIoSockets);
}

const ccl::SocketType *unirender::CCLShader::FindProperty(ccl::ShaderNode &node,const std::string &inputName)
{
	auto translatedInputName = TranslateInputName(node,inputName);
	auto *type = icycles::shader_node::get_type(node);
	auto n = icycles::node_type::get_input_count(*type);
	for(auto i=decltype(n){0u};i<n;++i)
	{
		auto &socketType = *icycles::node_type::get_input(*type,i);
		icycles::CString socketTypeName {};
		icycles::socket_type::get_name(socketType,&socketTypeName.cstr,socketTypeName.len);
		if(icycles::util::string_iequals(socketTypeName.cstr,translatedInputName.c_str()))
			return &socketType;
	}
	return nullptr;
}
ccl::ShaderInput *unirender::CCLShader::FindInput(ccl::ShaderNode &node,const std::string &inputName)
{
	// return node.input(ccl::ustring{inputName}); // Doesn't work in some cases for some reason
	auto translatedInputName = TranslateInputName(node,inputName);
	auto n = icycles::shader_node::get_input_count(node);
	for(auto i=decltype(n){0u};i<n;++i)
	{
		auto &input = *icycles::shader_node::get_input(node,i);
		auto &socketType = icycles::shader_input::get_socket_type(input);
		icycles::CString inputName {};
		icycles::socket_type::get_name(socketType,&inputName.cstr,inputName.len);
		if(icycles::util::string_iequals(inputName.cstr,translatedInputName.c_str()))
			return &input;
	}
	return nullptr;
}
ccl::ShaderOutput *unirender::CCLShader::FindOutput(ccl::ShaderNode &node,const std::string &outputName)
{
	// return node.output(ccl::ustring{outputName}); // Doesn't work in some cases for some reason
	auto n = icycles::shader_node::get_output_count(node);
	for(auto i=decltype(n){0u};i<n;++i)
	{
		auto &output = *icycles::shader_node::get_output(node,i);
		auto &socketType = icycles::shader_output::get_socket_type(output);
		icycles::CString curOutputName {};
		icycles::socket_type::get_name(socketType,&curOutputName.cstr,curOutputName.len);
		if(icycles::util::string_iequals(curOutputName.cstr,outputName.c_str()))
			return &output;
	}
	return nullptr;
}

std::string unirender::CCLShader::GetCurrentInternalNodeName() const {return "internal_" +std::to_string(icycles::shader_graph::get_node_count(m_cclGraph));}
#pragma optimize("",on)
