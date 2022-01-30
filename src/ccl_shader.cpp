/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2020 Florian Weischer
*/

#include "util_raytracing/shader.hpp"
#include "unirender/cycles/ccl_shader.hpp"
#include "util_raytracing/scene.hpp"
#include "util_raytracing/mesh.hpp"
#include "util_raytracing/exception.hpp"
#include "unirender/cycles/renderer.hpp"
#include <scene/shader.h>
#include <scene/shader_graph.h>
#include <scene/scene.h>
#include <scene/shader_nodes.h>
#include <OpenImageIO/ustring.h>

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
ccl::NodeVectorMathType unirender::cycles::to_ccl_math_type(unirender::nodes::vector_math::MathType type)
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

ccl::ustring unirender::cycles::to_ccl_type(unirender::ColorSpace space)
{
	switch(space)
	{
	case unirender::ColorSpace::Raw:
		return ccl::u_colorspace_raw;
	case unirender::ColorSpace::Srgb:
		return ccl::u_colorspace_srgb;
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
	cclShader.set_volume_sampling_method(ccl::VOLUME_SAMPLING_MULTIPLE_IMPORTANCE);

	ccl::ShaderGraph *graph = new ccl::ShaderGraph();
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
	auto *cclShader = new ccl::Shader{}; // Object will be removed automatically by cycles
	cclShader->name = desc.GetName();
	renderer.GetCclScene()->shaders.push_back(cclShader);
	return Create(renderer,*cclShader,desc,true);
}

unirender::CCLShader::CCLShader(cycles::Renderer &renderer,ccl::Shader &cclShader,ccl::ShaderGraph &cclShaderGraph)
	: m_renderer{renderer},m_cclShader{cclShader},m_cclGraph{cclShaderGraph}
{}

unirender::CCLShader::~CCLShader()
{
	if(umath::is_flag_set(m_flags,Flags::CCLShaderGraphOwnedByScene) == false)
		delete &m_cclGraph;
	if(umath::is_flag_set(m_flags,Flags::CCLShaderOwnedByScene) == false)
		delete &m_cclShader;
}

ccl::Shader *unirender::CCLShader::operator->() {return &m_cclShader;}
ccl::Shader *unirender::CCLShader::operator*() {return &m_cclShader;}

void unirender::CCLShader::DoFinalize(Scene &scene)
{
	BaseObject::DoFinalize(scene);
	m_flags |= Flags::CCLShaderGraphOwnedByScene | Flags::CCLShaderOwnedByScene;
	m_cclShader.set_graph(&m_cclGraph);
	m_cclShader.tag_update(m_renderer.GetCclScene());
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
					return imageTexNode->type->find_input(ccl::ustring{unirender::nodes::image_texture::IN_FILENAME});
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
		wrapper->imageTexNode->set_colorspace(ccl::u_colorspace_raw);

		auto *sep = static_cast<ccl::SeparateRGBNode*>(AddNode(unirender::NODE_SEPARATE_RGB));
		m_cclGraph.connect(FindOutput(*wrapper->imageTexNode,unirender::nodes::image_texture::OUT_COLOR),FindInput(*sep,unirender::nodes::separate_rgb::IN_COLOR));

		auto *cmb = static_cast<ccl::CombineRGBNode*>(AddNode(unirender::NODE_COMBINE_RGB));
		m_cclGraph.connect(FindOutput(*sep,unirender::nodes::separate_rgb::OUT_R),FindInput(*cmb,unirender::nodes::combine_rgb::IN_G));
		m_cclGraph.connect(FindOutput(*sep,unirender::nodes::separate_rgb::OUT_G),FindInput(*cmb,unirender::nodes::combine_rgb::IN_R));
		m_cclGraph.connect(FindOutput(*sep,unirender::nodes::separate_rgb::OUT_B),FindInput(*cmb,unirender::nodes::combine_rgb::IN_B));
		
		wrapper->normalMapNode = static_cast<ccl::NormalMapNode*>(AddNode(unirender::NODE_NORMAL_MAP));
		assert(wrapper->normalMapNode);
		wrapper->normalMapNode->set_space(ccl::NodeNormalMapSpace::NODE_NORMAL_MAP_TANGENT);

		auto *normIn = FindInput(*wrapper->normalMapNode,unirender::nodes::normal_map::IN_COLOR);
		m_cclGraph.connect(FindOutput(*cmb,unirender::nodes::combine_rgb::OUT_IMAGE),normIn);
		return wrapper;
	}
	return nullptr;
}

ccl::ShaderNode *unirender::CCLShader::AddNode(const std::string &typeName)
{
	auto *nodeType = ccl::NodeType::find(ccl::ustring{typeName});
	auto *snode = nodeType ? static_cast<ccl::ShaderNode*>(nodeType->create(nodeType)) : nullptr;
	if(snode == nullptr)
		return nullptr;

	auto name = GetCurrentInternalNodeName();
	snode->name = name;
	m_cclGraph.add(snode);
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
						auto *prop = FindProperty(*cclToSocket->first,cclToSocket->second);
						if(fromSocketDesc && fromSocketDesc->dataValue.value && prop)
							ApplySocketValue(*fromSocketDesc,*cclToSocket->first,*prop);
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
			m_cclGraph.connect(output,input);
		}
		return;
	}
	auto &typeName = desc.GetTypeName();
	if(typeName == "output")
	{
		// Output node already exists by default
		nodeToCclNode[&desc] = m_cclGraph.output();
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
			return node->type->find_input(ccl::ustring{name});
		}
		virtual ccl::ShaderNode *GetOutputNode() override {return node;}
		ccl::ShaderNode *node = nullptr;
	};
	auto *snode = AddNode(typeName);
	std::unique_ptr<unirender::CCLShader::BaseNodeWrapper> wrapper = nullptr;
	if(snode != nullptr)
	{
		wrapper = std::make_unique<CclNodeWrapper>();
		static_cast<CclNodeWrapper*>(wrapper.get())->node = snode;
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
		auto *input = wrapper->FindInput(pair.first,&node);
		if(input == nullptr)
			continue; // TODO
		ApplySocketValue(pair.second,*node,input->socket_type);
	}

	for(auto &pair : desc.GetProperties())
	{
		ccl::ShaderNode *node;
		auto *type = wrapper->FindProperty(pair.first,&node);
		if(type == nullptr)
			continue; // TODO
		ApplySocketValue(pair.second,*node,*type);
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

void unirender::CCLShader::ApplySocketValue(const NodeSocketDesc &sockDesc,ccl::Node &node,const ccl::SocketType &sockType)
{
	switch(sockDesc.dataValue.type)
	{
	case SocketType::Bool:
		static_assert(std::is_same_v<STBool,bool>);
		node.set(sockType,*static_cast<STBool*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Float:
		static_assert(std::is_same_v<STFloat,float>);
		node.set(sockType,*static_cast<STFloat*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Int:
		static_assert(std::is_same_v<STInt,int32_t>);
		node.set(sockType,*static_cast<STInt*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Enum:
		static_assert(std::is_same_v<STEnum,int32_t>);
		node.set(sockType,*static_cast<STEnum*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::UInt:
		static_assert(std::is_same_v<STUInt,ccl::uint>);
		node.set(sockType,*static_cast<STUInt*>(sockDesc.dataValue.value.get()));
		break;
	case SocketType::Color:
	case SocketType::Vector:
	case SocketType::Point:
	case SocketType::Normal:
	{
		static_assert(std::is_same_v<STColor,Vector3> && std::is_same_v<STVector,Vector3> && std::is_same_v<STPoint,Vector3> && std::is_same_v<STNormal,Vector3>);
		auto &v = *static_cast<STVector*>(sockDesc.dataValue.value.get());
		node.set(sockType,ccl::float3{v.x,v.y,v.z});
		break;
	}
	case SocketType::Point2:
	{
		static_assert(std::is_same_v<STPoint2,Vector2>);
		auto &v = *static_cast<STPoint2*>(sockDesc.dataValue.value.get());
		node.set(sockType,ccl::float2{v.x,v.y});
		break;
	}
	case SocketType::String:
	{
		static_assert(std::is_same_v<STString,std::string>);
		auto &v = *static_cast<std::string*>(sockDesc.dataValue.value.get());
		node.set(sockType,v.c_str());
		break;
	}
	case SocketType::Transform:
	{
		static_assert(std::is_same_v<STTransform,Mat4x3>);
		auto &v = *static_cast<Mat4x3*>(sockDesc.dataValue.value.get());
		node.set(sockType,ccl::Transform{
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
		node.set(sockType,cclArray);
		break;
	}
	case SocketType::ColorArray:
	{
		static_assert(std::is_same_v<STColorArray,std::vector<STColor>>);
		auto &v = *static_cast<std::vector<STColor>*>(sockDesc.dataValue.value.get());
		auto cclArray = to_ccl_array<Vector3,ccl::float3>(v,[](const Vector3 &v) -> ccl::float3 {return ccl::float3{v.x,v.y,v.z};});
		node.set(sockType,cclArray);
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
				nodeMath->set_math_type(ccl::NodeMathType::NODE_MATH_ADD);
				nodeMath->set_value1(0.f);
				nodeMath->set_value2(0.f);

				if(socketDesc.dataValue.value)
				{
					auto v = socketDesc.dataValue.ToValue<float>();
					if(v.has_value())
						nodeMath->set_value1(*v);
				}
				socketTranslation.input = {nodeMath,nodes::math::IN_VALUE1};
				socketTranslation.output = {nodeMath,nodes::math::OUT_VALUE};
			}
			else if(is_convertible_to(socketDesc.dataValue.type,SocketType::Vector))
			{
				auto *nodeVec = static_cast<ccl::VectorMathNode*>(AddNode(NODE_VECTOR_MATH));
				assert(nodeVec);
				nodeVec->set_math_type(ccl::NodeVectorMathType::NODE_VECTOR_MATH_ADD);
				nodeVec->set_vector1({0.f,0.f,0.f});
				nodeVec->set_vector2({0.f,0.f,0.f});

				if(socketDesc.dataValue.value)
				{
					auto v = socketDesc.dataValue.ToValue<Vector3>();
					if(v.has_value())
						nodeVec->set_vector1({v->x,v->y,v->z});
				}
				socketTranslation.input = {nodeVec,nodes::vector_math::IN_VECTOR1};
				socketTranslation.output = {nodeVec,nodes::vector_math::OUT_VECTOR};
			}
			else if(socketDesc.dataValue.type == unirender::SocketType::Closure)
			{
				auto *mix = static_cast<ccl::MixClosureNode*>(AddNode(NODE_MIX_CLOSURE));
				assert(mix);
				mix->set_fac(0.f);

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

const ccl::SocketType *unirender::CCLShader::FindProperty(ccl::ShaderNode &node,const std::string &inputName) const
{
	auto it = std::find_if(node.type->inputs.begin(),node.type->inputs.end(),[&inputName](const ccl::SocketType &socketType) {
		return ccl::string_iequals(socketType.name.string(),inputName);
	});
	return (it != node.type->inputs.end()) ? &*it : nullptr;
}
ccl::ShaderInput *unirender::CCLShader::FindInput(ccl::ShaderNode &node,const std::string &inputName)
{
	// return node.input(ccl::ustring{inputName}); // Doesn't work in some cases for some reason
	auto it = std::find_if(node.inputs.begin(),node.inputs.end(),[&inputName](const ccl::ShaderInput *shInput) {
		return ccl::string_iequals(shInput->socket_type.name.string(),inputName);
	});
	return (it != node.inputs.end()) ? *it : nullptr;
}
ccl::ShaderOutput *unirender::CCLShader::FindOutput(ccl::ShaderNode &node,const std::string &outputName)
{
	// return node.output(ccl::ustring{outputName}); // Doesn't work in some cases for some reason
	auto it = std::find_if(node.outputs.begin(),node.outputs.end(),[&outputName](const ccl::ShaderOutput *shOutput) {
		return ccl::string_iequals(shOutput->socket_type.name.string(),outputName);
	});
	return (it != node.outputs.end()) ? *it : nullptr;
}

std::string unirender::CCLShader::GetCurrentInternalNodeName() const {return "internal_" +std::to_string(m_cclGraph.nodes.size());}
#pragma optimize("",on)
