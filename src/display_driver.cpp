/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#include "unirender/cycles/display_driver.hpp"
#include <mathutil/color.h>
#include <util_image_buffer.hpp>
#pragma optimize("",off)
unirender::cycles::DisplayDriver::DisplayDriver(uint32_t width,uint32_t height)
	: m_width{width},m_height{height}
{
	m_imageBuffer = uimg::ImageBuffer::Create(width,height,uimg::Format::RGBA16);
}
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
	return static_cast<ccl::half4*>(m_imageBuffer->GetData());
}
void unirender::cycles::DisplayDriver::unmap_texture_buffer() {}
void unirender::cycles::DisplayDriver::clear()
{
	m_imageBuffer->Clear(Vector4{});
}
void unirender::cycles::DisplayDriver::draw(const Params &params)
{

}

////////////

unirender::cycles::OutputDriver::OutputDriver(uint32_t width,uint32_t height)
	: m_width{width},m_height{height}
{
	m_imageBuffer = uimg::ImageBuffer::Create(width,height,uimg::Format::RGBA32);
	m_tileData.resize(width *height);
}
void unirender::cycles::OutputDriver::write_render_tile(const Tile &tile)
{
	if(!tile.get_pass_pixels("combined",4,reinterpret_cast<float*>(m_tileData.data())))
		return;

	auto *data = static_cast<Vector4*>(m_imageBuffer->GetData());
	data += tile.offset.y *m_imageBuffer->GetWidth() +tile.offset.x;
	auto *srcData = m_tileData.data();
	for(auto y=decltype(tile.size.y){0};y<tile.size.y;++y)
	{
		memcpy(data,srcData,tile.size.x *tile.size.y *sizeof(Vector4));
		data += m_imageBuffer->GetWidth();
		srcData += tile.size.x;
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
