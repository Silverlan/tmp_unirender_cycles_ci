/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright (c) 2021 Silverlan
*/

#ifndef __UNIRENDER_CYCLES_DISPLAY_DRIVER_HPP__
#define __UNIRENDER_CYCLES_DISPLAY_DRIVER_HPP__

#include <session/display_driver.h>
#include <session/output_driver.h>
#include <util_image_types.hpp>
#include <mathutil/uvec.h>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <memory>
#include <vector>

namespace uimg {class ImageBuffer;};
namespace unirender {class TileManager;};
namespace unirender::cycles
{
	class BaseDriver
	{
	public:
		BaseDriver(const std::vector<std::pair<std::string,uimg::Format>> &passes,uint32_t width,uint32_t height);
		virtual ~BaseDriver()=default;
		std::shared_ptr<uimg::ImageBuffer> GetImageBuffer(const std::string &pass) const;
	protected:
		uint32_t m_width = 0;
		uint32_t m_height = 0;
		std::unordered_map<std::string,std::shared_ptr<uimg::ImageBuffer>> m_imageBuffers;
	};

	class DisplayDriver
		: public ccl::DisplayDriver,public BaseDriver
	{
	public:
		DisplayDriver(unirender::TileManager &tileManager,uint32_t width,uint32_t height);
		virtual ~DisplayDriver() override;
		virtual bool update_begin(const Params &params, int width, int height) override;
		virtual void update_end() override;
		virtual ccl::half4 *map_texture_buffer() override;
		virtual void unmap_texture_buffer() override;
		virtual void clear() override;
		virtual void draw(const Params &params) override;
		virtual void next_tile_begin() override {};

		void ResetTileWrittenFlag() {m_tileWritten = false;}
		bool WasTileWritten() const {return m_tileWritten;}
	private:
		void RunPostProcessing();
		unirender::TileManager &m_tileManager;
		std::atomic<bool> m_tileWritten = false;
		std::thread m_postProcessingThread;
		std::mutex m_postProcessingMutex;
		std::condition_variable m_postProcessingCondition;
		
		std::shared_ptr<uimg::ImageBuffer> m_mappedImageBuffer = nullptr;
		std::shared_ptr<uimg::ImageBuffer> m_pendingForPpImageBuffer = nullptr;
		std::atomic<bool> m_imageBufferReadyForPp = false;
		std::atomic<bool> m_ppThreadRunning = true;
	};

	class OutputDriver
		: public ccl::OutputDriver,public BaseDriver
	{
	public:
		OutputDriver(const std::vector<std::pair<std::string,uimg::Format>> &passes,uint32_t width,uint32_t height);
		/* Write tile once it has finished rendering. */
		virtual void write_render_tile(const Tile &tile) override;

		/* Update tile while rendering is in progress. Return true if any update
		* was performed. */
		virtual bool update_render_tile(const Tile & /* tile */) override;

		/* For baking, read render pass PASS_BAKE_PRIMITIVE and PASS_BAKE_DIFFERENTIAL
		* to determine which shading points to use for baking at each pixel. Return
		* true if any data was read. */
		virtual bool read_render_tile(const Tile & /* tile */) override;
	private:
		std::vector<Vector4> m_tileData;
	};
};

#endif
