#pragma once

#include "Layer.h"
#include <mapnik/map.hpp>
#include <mapnik/datasource_cache.hpp>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/image.hpp>
#include <mapnik/image_util.hpp>
#include <mapnik/image_view.hpp>
#include <mapnik/image_view_any.hpp>
#include <mapnik/cairo_io.hpp>
#include <mutex>
#include <condition_variable>


class MapnikLayer;

class MapnikMap
{
public:
	MapnikMap(MapnikLayer &map, std::unique_ptr<mapnik::Map> mapnikMap)
	:
		m_map(map),
		m_mapnikMap(std::move(mapnikMap))
	{
	}

	MapnikMap(MapnikMap &&other)
	:
		m_map(other.m_map),
		m_mapnikMap(std::move(other.m_mapnikMap))
	{
	}

	~MapnikMap();

	void zoom_to_box(const mapnik::box2d<double> &box)
	{
		m_mapnikMap->zoom_to_box(box);
	}

	operator mapnik::Map& ()
	{
		return *m_mapnikMap;
	}

private:

	MapnikLayer &m_map;
	std::unique_ptr<mapnik::Map> m_mapnikMap;
};


class MapnikLayer: public Layer
{
public:

	MapnikLayer(
		const std::string &stateDir,
		const std::string &name,
		const std::string &xmlFile,
		int metaTiles,
		int poolSize,
		int workerPoolSize);

	void submitRequest(
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		Layer::StateChangeCallback stateChangeCallback) override;

	MapnikMap acquireMap();

	int m_metaTiles {1};
	int m_metaSize {m_tileSize * m_metaTiles + m_tileSize};
	int m_poolSize {1};

	std::mutex m_mapMutex;
	std::condition_variable m_mapCondition;
	std::vector<std::unique_ptr<mapnik::Map>> m_maps;
};
