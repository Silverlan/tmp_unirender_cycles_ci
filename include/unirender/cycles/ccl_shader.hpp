/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2020 Florian Weischer
*/

#ifndef __CCL_SHADER_HPP__
#define __CCL_SHADER_HPP__

#include <util_raytracing/shader.hpp>
#include <render/nodes.h>
#include <sharedutils/datastream.h>
#include <sharedutils/util_virtual_shared_from_this.hpp>

namespace ccl
{
	class Scene; class Shader; class ShaderGraph; class ShaderNode; class ShaderInput; class ShaderOutput;
	enum NodeMathType : int32_t;
	enum AttributeStandard : int32_t;
};

namespace unirender
{
	namespace cycles {
		class Renderer;
		ccl::NodeMathType to_ccl_type(unirender::nodes::math::MathType type);
		ccl::NodeVectorMathType to_ccl_math_type(unirender::nodes::vector_math::MathType type);
		ccl::ustring to_ccl_type(unirender::ColorSpace space);
		ccl::NodeEnvironmentProjection to_ccl_type(unirender::EnvironmentProjection projection);
		ccl::ClosureType to_ccl_type(unirender::ClosureType type);
		ccl::ImageAlphaType to_ccl_type(unirender::nodes::image_texture::AlphaType type);
		ccl::InterpolationType to_ccl_type(unirender::nodes::image_texture::InterpolationType type);
		ccl::ExtensionType to_ccl_type(unirender::nodes::image_texture::ExtensionType type);
		ccl::NodeImageProjection to_ccl_type(unirender::nodes::image_texture::Projection type);
		ccl::NodeMappingType to_ccl_type(unirender::nodes::mapping::Type type);
		ccl::NodeNormalMapSpace to_ccl_type(unirender::nodes::normal_map::Space space);
		ccl::NodeMix to_ccl_type(unirender::nodes::mix::Mix mix);
		ccl::NodeVectorTransformConvertSpace to_ccl_type(unirender::nodes::vector_transform::ConvertSpace convertSpace);
	};
	struct GroupSocketTranslation
	{
		std::pair<ccl::ShaderNode*,std::string> input;
		std::pair<ccl::ShaderNode*,std::string> output;
	};
	using GroupSocketTranslationTable = std::unordered_map<Socket,GroupSocketTranslation,SocketHasher>;
	class CCLShader
		: public std::enable_shared_from_this<CCLShader>,
		public BaseObject
	{
	public:
		enum class Flags : uint8_t
		{
			None = 0u,
			
			CCLShaderOwnedByScene = 1u,
			CCLShaderGraphOwnedByScene = CCLShaderOwnedByScene<<1u
		};
		static std::shared_ptr<CCLShader> Create(cycles::Renderer &renderer,const GroupNodeDesc &desc);
		static std::shared_ptr<CCLShader> Create(cycles::Renderer &renderer,ccl::Shader &cclShader,const GroupNodeDesc &desc,bool useCache=false);
		static ccl::ShaderInput *FindInput(ccl::ShaderNode &node,const std::string &inputName);
		static ccl::ShaderOutput *FindOutput(ccl::ShaderNode &node,const std::string &outputName);

		~CCLShader();
		void InitializeNodeGraph(const GroupNodeDesc &desc);

		ccl::Shader *operator->();
		ccl::Shader *operator*();
	protected:
		CCLShader(cycles::Renderer &renderer,ccl::Shader &cclShader,ccl::ShaderGraph &cclShaderGraph);
		virtual void DoFinalize(Scene &scene) override;
		void InitializeNode(const NodeDesc &desc,std::unordered_map<const NodeDesc*,ccl::ShaderNode*> &nodeToCclNode,const GroupSocketTranslationTable &groupIoSockets);
		void ConvertGroupSocketsToNodes(const GroupNodeDesc &groupDesc,GroupSocketTranslationTable &outGroupIoSockets);
		const ccl::SocketType *FindProperty(ccl::ShaderNode &node,const std::string &inputName) const;
		void ApplySocketValue(const NodeSocketDesc &sockDesc,ccl::Node &node,const ccl::SocketType &sockType);
		std::string GetCurrentInternalNodeName() const;
	private:
		struct BaseNodeWrapper
		{
			virtual ccl::ShaderInput *FindInput(const std::string &name,ccl::ShaderNode **outNode)=0;
			virtual ccl::ShaderOutput *FindOutput(const std::string &name,ccl::ShaderNode **outNode)=0;
			virtual const ccl::SocketType *FindProperty(const std::string &name,ccl::ShaderNode **outNode)=0;
			virtual ccl::ShaderNode *GetOutputNode()=0;
			virtual ~BaseNodeWrapper()=default;
		};
		std::unique_ptr<BaseNodeWrapper> ResolveCustomNode(const std::string &typeName);
		ccl::ShaderNode *AddNode(const std::string &type);
		ccl::Shader &m_cclShader;
		ccl::ShaderGraph &m_cclGraph;
		Flags m_flags = Flags::None;
		cycles::Renderer &m_renderer;
	};
};
REGISTER_BASIC_BITWISE_OPERATORS(unirender::CCLShader::Flags)

#endif
