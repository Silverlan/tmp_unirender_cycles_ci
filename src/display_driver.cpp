/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#include "unirender/cycles/display_driver.hpp"
#include <util_raytracing/tilemanager.hpp>
#include <mathutil/color.h>
#include <sharedutils/util_string.h>
#include <sharedutils/util.h>
#include <util_image_buffer.hpp>
#pragma optimize("",off)
unirender::cycles::BaseDriver::BaseDriver(const std::vector<std::pair<std::string,uimg::Format>> &passes,uint32_t width,uint32_t height)
	: m_width{width},m_height{height}
{
	m_imageBuffers.reserve(passes.size());
	for(auto &pair : passes)
	{
		auto imgBuf = uimg::ImageBuffer::Create(width,height,pair.second);
		m_imageBuffers[pair.first] = imgBuf;
	}
}
std::shared_ptr<uimg::ImageBuffer> unirender::cycles::BaseDriver::GetImageBuffer(const std::string &pass) const
{
	auto it = m_imageBuffers.find(pass);
	return (it != m_imageBuffers.end()) ? it->second : nullptr;
}

////////////

unirender::cycles::DisplayDriver::DisplayDriver(unirender::TileManager &tileManager,uint32_t width,uint32_t height)
	: BaseDriver{{{"combined",uimg::Format::RGBA16}},width,height},m_tileManager{tileManager}
{}
bool unirender::cycles::DisplayDriver::update_begin(const Params &params, int width, int height)
{
	if(width != m_width || height != m_height)
		return false;
	return true;
}
void unirender::cycles::DisplayDriver::update_end()
{

}
ccl::half4 *unirender::cycles::DisplayDriver::map_texture_buffer()
{
	static_assert(sizeof(ccl::half4) == sizeof(uint16_t) *4);
	return static_cast<ccl::half4*>(GetImageBuffer("combined")->GetData());
}
void unirender::cycles::DisplayDriver::unmap_texture_buffer()
{
	uint32_t tileIndex = 0;

	auto &inputTile = m_tileManager.GetInputTiles()[tileIndex];
	inputTile.x = 0;
	inputTile.y = 0;
	inputTile.w = m_width;
	inputTile.h = m_height;
	inputTile.index = tileIndex;
	inputTile.sample = 1;
	inputTile.flags |= unirender::TileManager::TileData::Flags::HDRData;
	inputTile.data.resize(inputTile.w *inputTile.h *sizeof(uint16_t) *4);
	auto imgBuf = GetImageBuffer("combined");
	imgBuf->Flip(true,true);
	auto *data = imgBuf->GetData();
	memcpy(inputTile.data.data(),data,util::size_of_container(inputTile.data));
	m_tileManager.ApplyPostProcessingForProgressiveTile(inputTile);
	m_tileManager.AddRenderedTile(std::move(inputTile));

	m_tileWritten = true;
}
void unirender::cycles::DisplayDriver::clear()
{
	auto imgBuf = GetImageBuffer("combined");
	auto *data = imgBuf->GetData();
	memset(data,0,imgBuf->GetSize());
}
void unirender::cycles::DisplayDriver::draw(const Params &params)
{

}

////////////

unirender::cycles::OutputDriver::OutputDriver(const std::vector<std::pair<std::string,uimg::Format>> &passes,uint32_t width,uint32_t height)
	: BaseDriver{passes,width,height}
{
	m_tileData.resize(width *height);
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
}

bool unirender::cycles::OutputDriver::update_render_tile(const Tile & tile )
{
	return false;
}

bool unirender::cycles::OutputDriver::read_render_tile(const Tile & /* tile */)
{
	return false;
}
#pragma optimize("",on)
