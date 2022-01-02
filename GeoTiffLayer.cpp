#include "GeoTiffLayer.h"
#include "TerrainRequest.h"
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <boost/filesystem.hpp>

GeoTiffLayer::GeoTiffLayer (
	const std::string &stateDir,
	const std::string &name,
	const std::string &terrainDir,
	int workerPoolSize)
:
	Layer(stateDir, name, "png", workerPoolSize),
	m_terrainDatasets(initializeTerrain(terrainDir))
{

}

void GeoTiffLayer::submitRequest(
	const Napi::Promise::Deferred &deferred,
	const Napi::ThreadSafeFunction &callback,
	int x,
	int y,
	int z,
	Layer::StateChangeCallback stateChangeCallback)
{
	auto request = std::make_shared<TerrainRequest>(shared_from_this(), deferred, callback, x, y, z, m_root,
		stateChangeCallback);

	m_queue.addRenderRequest(request);
}

std::vector<Dataset> GeoTiffLayer::initializeTerrain (const std::string &terrainDir)
{
	std::vector<Dataset> terrainDatasets;

    GDALAllRegister();

	for (auto &entry: boost::filesystem::directory_iterator(terrainDir))
	{
		auto dataset = std::unique_ptr<GDALDataset>(static_cast<GDALDataset *>(GDALOpen (entry.path ().c_str (), GA_ReadOnly)));

		if (dataset)
		{
			double transform[6];
			auto result = dataset->GetGeoTransform (transform);

			if (result == CE_None || result == CE_Debug || result == CE_Warning)
			{
				Dataset ds;

				ds.m_filename = entry.path().c_str ();

				ds.m_pixelSizeX = transform[1];
				ds.m_pixelSizeY = -transform[5];
				ds.m_numberOfBands = dataset->GetRasterCount();
				ds.m_rasterSizeX = dataset->GetRasterXSize();
				ds.m_rasterSizeY = dataset->GetRasterYSize();

				ds.m_extent = mapnik::box2d<double>(
						transform[0], transform[3] - (ds.m_rasterSizeY * transform[4] + ds.m_rasterSizeY * ds.m_pixelSizeY),
						transform[0] + ds.m_rasterSizeX * ds.m_pixelSizeX + ds.m_rasterSizeX * transform[2],
						transform[3]);

				terrainDatasets.push_back(ds);
			}
			else
			{
				std::cerr << "GetGeoTransform failed" << std::endl;
			}
		}
		else
		{
			std::cerr << "Data set allocation failed\n";
		}
	}

	return terrainDatasets;
}


