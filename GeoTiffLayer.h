#pragma once

#include "Layer.h"

class Dataset
{
public:
	std::string m_filename;
	mapnik::box2d<double> m_extent;
	double m_pixelSizeX;
	double m_pixelSizeY;
	int m_numberOfBands;
	int m_rasterSizeX;
	int m_rasterSizeY;
};

class GeoTiffLayer: public Layer
{
public:

	GeoTiffLayer(
		const std::string &stateDir,
		const std::string &name,
		const std::string &terrainDir,
		int workerPoolSize);

	std::vector<Dataset> m_terrainDatasets;

	void submitRequest(
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		Layer::StateChangeCallback stateChangeCallback) override;

private:

	std::vector<Dataset> initializeTerrain (const std::string &terrainDir);
};
