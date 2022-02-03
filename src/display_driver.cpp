/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#include "unirender/cycles/display_driver.hpp"
#include <mathutil/color.h>
#include <sharedutils/util_string.h>
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

unirender::cycles::DisplayDriver::DisplayDriver(uint32_t width,uint32_t height)
	: BaseDriver{{{"combined",uimg::Format::RGBA16}},width,height}
{}
void unirender::cycles::DisplayDriver::next_tile_begin()
{

}
bool unirender::cycles::DisplayDriver::update_begin(const Params &params, int width, int height)
{
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
void unirender::cycles::DisplayDriver::unmap_texture_buffer() {}
void unirender::cycles::DisplayDriver::clear()
{
	GetImageBuffer("combined")->Clear(Vector4{});
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
		auto szPerPixel = imgBuf->GetPixelSize();
		auto *data = static_cast<uint8_t*>(imgBuf->GetData());
		data += (tile.offset.y *imgBuf->GetWidth() +tile.offset.x) *szPerPixel;
		data += (imgBuf->GetWidth() *(tile.size.y -1)) *szPerPixel; // We'll flip the image vertically by starting at the bottom
		auto *srcData = reinterpret_cast<uint8_t*>(m_tileData.data());
		for(auto y=decltype(tile.size.y){0};y<tile.size.y;++y)
		{
			memcpy(data,srcData,tile.size.x *szPerPixel);
			data -= imgBuf->GetWidth() *szPerPixel;
			srcData += tile.size.x *szPerPixel;
		}
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
