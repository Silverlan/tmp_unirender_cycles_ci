/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#include "unirender/cycles/display_driver.hpp"
#include <util_raytracing/tilemanager.hpp>
#include <util_raytracing/denoise.hpp>
#include <util_image.hpp>
#include <mathutil/color.h>
#include <sharedutils/util_string.h>
#include <sharedutils/util.h>
#include <sharedutils/util_baking.hpp>
#include <fsys/filesystem.h>
#include <fsys/ifile.hpp>
#include <util_image_buffer.hpp>
#pragma optimize("",off)
static void dump_image_file(const std::string &name,uimg::ImageBuffer &imgBuf)
{
	auto f = filemanager::open_file("temp/cycles_driver_output_" +name +".hdr",filemanager::FileMode::Write | filemanager::FileMode::Binary);
	if(f)
	{
		fsys::File fp {f};
		uimg::save_image(fp,imgBuf,uimg::ImageFormat::HDR,1.f);
	}
}

unirender::cycles::BaseDriver::BaseDriver(uint32_t width,uint32_t height)
	: m_width{width},m_height{height}
{}

////////////

unirender::cycles::DisplayDriver::DisplayDriver(unirender::TileManager &tileManager,uint32_t width,uint32_t height)
	: BaseDriver{width,height},m_tileManager{tileManager}
{
	m_postProcessingThread = std::thread{[this]() {
		while(m_ppThreadRunning)
			RunPostProcessing();
	}};

	util::set_thread_name(m_postProcessingThread,"cycles_display_driver_pp");
}
unirender::cycles::DisplayDriver::~DisplayDriver()
{
	m_ppThreadRunning = false;
	m_postProcessingCondition.notify_one();
	m_postProcessingThread.join();
}
void unirender::cycles::DisplayDriver::UpdateTileResolution(uint32_t tileWidth,uint32_t tileHeight)
{
	if(tileWidth == m_tileWidth && tileHeight == m_tileHeight)
		return;
	m_tileWidth = tileWidth;
	m_tileHeight = tileHeight;
	m_numTilesX = m_width /m_tileWidth;
	if((m_width %m_tileWidth) > 0)
		++m_numTilesX;
	auto numTilesY = m_height /m_tileHeight;
	if((m_height %m_tileHeight) > 0)
		++numTilesY;
	auto tileCount = m_numTilesX *numTilesY;
	m_tmpImageBuffers.reserve(tileCount);
	m_mappedTileImageBuffers.reserve(tileCount);
	m_pendingForPpTileImageBuffers.reserve(tileCount);
	for(auto i=decltype(tileCount){0u};i<tileCount;++i)
	{
		m_tmpImageBuffers.push_back(uimg::ImageBuffer::Create(static_cast<void*>(nullptr),tileWidth,tileHeight,uimg::Format::RGBA16));
		m_mappedTileImageBuffers.push_back(uimg::ImageBuffer::Create(tileWidth,tileHeight,uimg::Format::RGBA16));
		m_pendingForPpTileImageBuffers.push_back(uimg::ImageBuffer::Create(tileWidth,tileHeight,uimg::Format::RGBA16));
	}
}
uint32_t unirender::cycles::DisplayDriver::GetTileIndex(uint32_t x,uint32_t y) const
{
	auto numTilesX = m_numTilesX;
	return (y /m_tileHeight) *numTilesX +(x /m_tileWidth);
}
bool unirender::cycles::DisplayDriver::update_begin(const Params &params, int effectiveWidth, int effectiveHeight)
{
	auto inBounds = (params.full_offset.x >= 0 && params.full_offset.y >= 0 && params.full_offset.x +effectiveWidth <= m_width && params.full_offset.y +effectiveHeight <= m_height);
	assert(inBound);
	if(!inBounds)
		return false;
	m_mappedSize = {effectiveWidth,effectiveHeight};
	m_mappedOffset = {params.full_offset.x,params.full_offset.y};
	m_mappedTileIndex = GetTileIndex(params.full_offset.x,params.full_offset.y);
	return true;
}
void unirender::cycles::DisplayDriver::update_end()
{
	m_mappedSize = {0,0};
	m_mappedOffset = {0,0};
	m_mappedTileIndex = 0;
}
void unirender::cycles::DisplayDriver::RunPostProcessing()
{
	struct TileData
	{
		TileInfo info;
		std::vector<uint8_t> data;
	};
	std::queue<TileData> tileInfos;
	{ // Note: This has to be scoped!
		std::unique_lock<std::mutex> lock {m_postProcessingMutex};
		m_postProcessingCondition.wait(lock,[this]() -> bool {return !m_imageBufferReadyForPp.empty() || !m_ppThreadRunning;});
		if(!m_ppThreadRunning)
			return;
		while(!m_imageBufferReadyForPp.empty())
		{
			auto tileInfo = m_imageBufferReadyForPp.front();
			m_imageBufferReadyForPp.pop();

			auto &src = *m_pendingForPpTileImageBuffers[tileInfo.tileIndex];
			TileData tileData {};
			tileData.info = tileInfo;
			tileData.data.resize(tileInfo.effectiveSize.x *tileInfo.effectiveSize.y *src.GetPixelSize());
			memcpy(tileData.data.data(),src.GetData(),util::size_of_container(tileData.data));
			tileInfos.push(std::move(tileData));
		}
	}

	while(!tileInfos.empty())
	{
		auto &tileData = tileInfos.front();
		auto &tileInfo = tileData.info;

		auto &inputTile = m_tileManager.GetInputTiles()[tileInfo.tileIndex];
		inputTile.x = tileInfo.tileOffset.x;
		inputTile.y = tileInfo.tileOffset.y;
		inputTile.w = tileInfo.effectiveSize.x;
		inputTile.h = tileInfo.effectiveSize.y;
		inputTile.index = tileInfo.tileIndex;
		inputTile.sample = 1; // Not accurate, but doesn't matter here
		inputTile.flags |= unirender::TileManager::TileData::Flags::HDRData;
		inputTile.data = std::move(tileData.data);
		
		auto &imgBuf = m_tmpImageBuffers[tileInfo.tileIndex];
		imgBuf->Reset(inputTile.data.data(),inputTile.w,inputTile.h,imgBuf->GetFormat());
		imgBuf->Flip(true,true);

		// This is deprecated; Denoising is now handled through Cycles
		static auto denoise = false;
		if(denoise)
		{
			denoise::Info denoiseInfo {};
			denoiseInfo.hdr = true;
			denoiseInfo.width = inputTile.w;
			denoiseInfo.height = inputTile.h;
			unirender::denoise::ImageData output {};
			output.data = inputTile.data.data();
			output.format = imgBuf->GetFormat();
			unirender::denoise::ImageInputs inputs {};
			inputs.beautyImage = output;
			denoise::denoise(denoiseInfo,inputs,output);
		}
		static auto enablePp = true;
		if(enablePp)
			m_tileManager.ApplyPostProcessingForProgressiveTile(inputTile);
		m_tileManager.AddRenderedTile(std::move(inputTile));

		tileInfos.pop();
	}

	m_tileWritten = true;
}
ccl::half4 *unirender::cycles::DisplayDriver::map_texture_buffer()
{
	static_assert(sizeof(ccl::half4) == sizeof(uint16_t) *4);
	return static_cast<ccl::half4*>(m_mappedTileImageBuffers[m_mappedTileIndex]->GetData());
}
void unirender::cycles::DisplayDriver::unmap_texture_buffer()
{
	// We want to keep the overhead here to a bare minimum, in order to avoid
	// stalling the Cycles renderer.
	// To do that, we'll postpone the post-processing to a separate thread, and let
	// Cycles continue with a different image buffer immediately.
	std::scoped_lock lock {m_postProcessingMutex};

	/*static auto debugDumpAsFile = false;
	if(debugDumpAsFile)
	{
		debugDumpAsFile = false;
		dump_image_file("combined",*m_mappedImageBuffer);
	}*/

	uint32_t tileIndex = m_mappedTileIndex;
	auto &mappedImageBuffer = m_mappedTileImageBuffers[tileIndex];
	auto &pendingForPpImageBuffer = m_pendingForPpTileImageBuffers[tileIndex];
	std::swap(mappedImageBuffer,pendingForPpImageBuffer);
	m_imageBufferReadyForPp.push({tileIndex,m_mappedOffset,m_mappedSize});
	m_postProcessingCondition.notify_one();
}
void unirender::cycles::DisplayDriver::clear()
{
	// TODO: m_pendingForPpImageBuffer should be cleared as well?
	for(auto &imgBuf : m_mappedTileImageBuffers)
	{
		auto *data = imgBuf->GetData();
		memset(data,0,imgBuf->GetSize());
	}
}
void unirender::cycles::DisplayDriver::next_tile_begin() {}
void unirender::cycles::DisplayDriver::draw(const Params &params)
{

}

////////////

unirender::cycles::OutputDriver::OutputDriver(const std::vector<std::pair<std::string,uimg::Format>> &passes,uint32_t width,uint32_t height)
	: BaseDriver{width,height},m_passes{passes}
{
	m_tileData.resize(width *height);
	Reset();
}
void unirender::cycles::OutputDriver::Reset()
{
	m_imageBuffers.clear();
	m_imageBuffers.reserve(m_passes.size());
	for(auto &pair : m_passes)
	{
		auto imgBuf = uimg::ImageBuffer::Create(m_width,m_height,pair.second);
		m_imageBuffers[pair.first] = imgBuf;
	}
}
void unirender::cycles::OutputDriver::DebugDumpImages()
{
	for(auto &pair : m_imageBuffers)
		dump_image_file(pair.first,*pair.second);
}
std::shared_ptr<uimg::ImageBuffer> unirender::cycles::OutputDriver::GetImageBuffer(const std::string &pass) const
{
	auto it = m_imageBuffers.find(pass);
	return (it != m_imageBuffers.end()) ? it->second : nullptr;
}
void unirender::cycles::OutputDriver::write_render_tile(const Tile &tile)
{
	for(auto &pair : m_imageBuffers)
	{
		auto &imgBuf = pair.second;
		assert(imgBuf->IsFloatFormat());
		if(!tile.get_pass_pixels(pair.first,imgBuf->GetChannelCount(),reinterpret_cast<float*>(m_tileData.data())))
			return;
		memcpy(imgBuf->GetData(),m_tileData.data(),util::size_of_container(m_tileData));
	}

	static auto debugDumpAsFile = false;
	if(debugDumpAsFile)
	{
		debugDumpAsFile = false;
		DebugDumpImages();
	}
}

bool unirender::cycles::OutputDriver::update_render_tile(const Tile & tile )
{
	//write_render_tile(tile);
	return false;
}

static float intToFloat(int32_t i)
{
	union {
		int i;
		float f;
	} u;
	u.i = i;
	return u.f;
}
void unirender::cycles::OutputDriver::SetBakeData(const util::baking::BakeDataView &bakeData) {m_bakeData = &bakeData;}
bool unirender::cycles::OutputDriver::read_render_tile(const Tile &tile)
{
	if(!m_bakeData)
		return false;
	auto &bakeData = *m_bakeData;

	auto w = tile.size.x;
	auto h = tile.size.y;
	auto x = tile.offset.x;
	auto y = tile.offset.y;
	std::vector<float> primitiveData;
	std::vector<float> differentialData;
	primitiveData.resize(w *h *4);
	differentialData.resize(w *h *4);

	for(int ty = 0; ty < h; ty++)
	{
		size_t offset = ty * w * 4;
		float *primitive = primitiveData.data() + offset;
		float *differential = differentialData.data() + offset;

		size_t bake_offset = (y + ty) * bakeData.bakeImageWidth + x;
		const auto *bake_pixel = bakeData.bakePixels + bake_offset;

		for (int tx = 0; tx < w; tx++)
		{
			if (bake_pixel->objectId != bakeData.objectId)
			{
				primitive[0] = intToFloat(-1);
				primitive[1] = intToFloat(-1);
			}
			else
			{
				primitive[0] = intToFloat(bake_pixel->seed);
				primitive[1] = intToFloat(bake_pixel->primitiveId);
				primitive[2] = bake_pixel->uv[0];
				primitive[3] = bake_pixel->uv[1];

				differential[0] = bake_pixel->du_dx;
				differential[1] = bake_pixel->du_dy;
				differential[2] = bake_pixel->dv_dx;
				differential[3] = bake_pixel->dv_dy;
			}

			primitive += 4;
			differential += 4;
			bake_pixel++;
		}
	}

	auto imgPrimitive = uimg::ImageBuffer::Create(primitiveData.data(),w,h,uimg::Format::RGBA32,true);
	auto imgDifferential = uimg::ImageBuffer::Create(differentialData.data(),w,h,uimg::Format::RGBA32,true);
	tile.set_pass_pixels("BakePrimitive",imgPrimitive->GetChannelCount(),reinterpret_cast<float*>(imgPrimitive->GetData()));
	tile.set_pass_pixels("BakeDifferential",imgDifferential->GetChannelCount(),reinterpret_cast<float*>(imgDifferential->GetData()));
	return true;
}
#pragma optimize("",on)
