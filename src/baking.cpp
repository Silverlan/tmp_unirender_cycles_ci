/*
* Copyright 2011-2013 Blender Foundation
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "unirender/cycles/baking.hpp"
#include "util_raytracing/mesh.hpp"
#include "util_raytracing/object.hpp"
#include "util_raytracing/scene.hpp"
#include "unirender/cycles/renderer.hpp"
#include "icycles.hpp"
#include <sharedutils/util_baking.hpp>

void unirender::baking::prepare_bake_data(
	const cycles::Renderer &renderer,unirender::Object &o,util::baking::BakePixel *pixelArray,uint32_t numPixels,
	uint32_t imgWidth,uint32_t imgHeight,bool useLightmapUvs
)
{
	auto objId = renderer.FindCCLObjectId(*renderer.FindCclObject(o));
	assert(objId.has_value());
	
	auto &mesh = o.GetMesh();
	auto *cclMesh = renderer.FindCclMesh(mesh);
	auto &tris = icycles::mesh::get_triangles(*cclMesh);
	auto &uvs = useLightmapUvs ? mesh.GetLightmapUvs() : mesh.GetUvs();
	auto numTris = tris.size() /3;

	util::baking::MeshInterface meshInterface {};
	meshInterface.getTriangle = [&tris](uint32_t idx) -> util::baking::Triangle {
		auto offset = idx *3;
		return {
			static_cast<uint32_t>(tris[offset]),
			static_cast<uint32_t>(tris[offset +1]),
			static_cast<uint32_t>(tris[offset +2])
		};
	};
	meshInterface.getUv = [&uvs](uint32_t vertIdx) -> util::baking::Uv {
		auto &uv = uvs[vertIdx];
		return {uv.x,uv.y};
	};

	util::baking::BakeDataView bd;
	bd.bakePixels = pixelArray;

	auto &zspan = bd.span;
	zspan.rectx = imgWidth;
	zspan.recty = imgHeight;

	zspan.span1.resize(zspan.recty);
	zspan.span2.resize(zspan.recty);

	util::baking::prepare_bake_pixel_data(
		bd,*objId,meshInterface,numTris,
		imgWidth,imgHeight
	);
}
