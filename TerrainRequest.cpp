#include "TerrainRequest.h"
#include "GeoTiffLayer.h"
#include "Utilities.h"
#include "TileCache.h"
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <png++/png.hpp>
#include <sstream>
#include <memory>

TerrainRequest::TerrainRequest (
	const std::shared_ptr<Layer> &layer,
	const Napi::Promise::Deferred &deferred,
	const Napi::ThreadSafeFunction &callback,
	int x,
	int y,
	int z,
	const std::string &fileSystemRoot,
	RenderRequest::StateChangeCallback stateChange)
:
	RenderRequest(layer, RenderRequest::Type::Terrain, deferred, callback, x, y, z, 1, 0, fileSystemRoot, stateChange)
{
}

std::vector<std::shared_ptr<Tile>> TerrainRequest::render ()
{
  std::vector<std::shared_ptr<Tile>> result;
	bool dataSetFound = false;
	const int tileArea = m_layer->m_tileSize * m_layer->m_tileSize;

	std::vector<uint8_t> buffer;
	buffer.resize(tileArea);
	std::fill(buffer.begin(), buffer.end(), 0xb5);

	auto box = getBoundingBox ();

	for (const auto &ds: static_cast<GeoTiffLayer *>(m_layer.get())->m_terrainDatasets)
	{
		auto intersect = ds.m_extent.intersect(box);

		if (intersect.valid())
		{
			auto dataset = std::unique_ptr<GDALDataset>(static_cast<GDALDataset *>(GDALOpen (ds.m_filename.c_str (), GA_ReadOnly)));

			if (dataset)
			{
				dataSetFound = true;

				auto srcX = (intersect.minx() - ds.m_extent.minx()) / ds.m_pixelSizeX;
				auto srcY = (ds.m_extent.maxy() - intersect.maxy()) / ds.m_pixelSizeY;
				auto srcWidth = (intersect.maxx() - intersect.minx()) / ds.m_pixelSizeX;
				auto srcHeight = (intersect.maxy() - intersect.miny()) / ds.m_pixelSizeY;

				int dstX = std::lround(((intersect.minx() - box.minx()) * m_layer->m_tileSize) / box.width ());
				int dstY = std::lround(((box.maxy() - intersect.maxy()) * m_layer->m_tileSize) / box.height ());
				int dstWidth = std::lround(((intersect.maxx() - intersect.minx()) * m_layer->m_tileSize) / box.width ());
				int dstHeight = std::lround(((intersect.maxy() - intersect.miny()) * m_layer->m_tileSize) / box.height ());

				for (int i = 0; i < ds.m_numberOfBands; i++)
				{
					auto band = dataset->GetRasterBand(i + 1);

					if (band)
					{
						auto color = band->GetColorInterpretation ();

						if (color == GCI_GrayIndex)
						{
							GDALRasterIOExtraArg extraArg {};

							extraArg.nVersion = RASTERIO_EXTRA_ARG_CURRENT_VERSION;
							extraArg.eResampleAlg = GRIORA_Bilinear;
							extraArg.bFloatingPointWindowValidity = true;
							extraArg.dfXOff = srcX;
							extraArg.dfYOff = srcY;
							extraArg.dfXSize = srcWidth;
							extraArg.dfYSize = srcHeight;

							auto result = band->RasterIO(
									GF_Read,
									std::lround(srcX),
									std::lround(srcY),
									std::lround(srcWidth),
									std::lround(srcHeight),
									buffer.data () + dstY * m_layer->m_tileSize + dstX,
									dstWidth,
									dstHeight,
									GDT_Byte,
									1,
									m_layer->m_tileSize,
									&extraArg);

							if (result != CE_None)
							{
								std::cerr << CPLGetLastErrorMsg() << std::endl;
							}
						}
						else
						{
							std::cerr << "Unsupported color interpretation: " << color << std::endl;
						}
					}
				}
			}
		}
	}

	if (dataSetFound)
	{
		png::image<png::gray_pixel> image(m_layer->m_tileSize, m_layer->m_tileSize);

		for (int y2 = 0; y2 < m_layer->m_tileSize; y2++)
		{
			for (int x2 = 0; x2 < m_layer->m_tileSize; x2++)
			{
				auto value = buffer[y2 * m_layer->m_tileSize + x2];

				image[y2][x2] = png::gray_pixel(value);
			}
		}

    std::stringstream ss;
    image.write_stream(ss);

    result.push_back(m_layer->saveTile(m_x, m_y, m_z, ss.str()));
	}

	return result;
}
