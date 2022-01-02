#include "MapnikLayer.h"
#include "DetailRequest.h"
#include <mapnik/load_map.hpp>

MapnikLayer::MapnikLayer (
	const std::string &stateDir,
	const std::string &name,
	const std::string &xmlFile,
	int metaTiles,
	int poolSize,
	int workerPoolSize)
:
	Layer(stateDir, name, "png", workerPoolSize),
	m_metaTiles (metaTiles),
	m_metaSize (m_tileSize * m_metaTiles + m_tileSize),
	m_poolSize (poolSize)
{
	mapnik::datasource_cache::instance().register_datasources("/usr/lib/mapnik/3.0/input");

	for (int i = 0; i < m_poolSize; i++) {
		auto map = std::make_unique<mapnik::Map>(m_metaSize, m_metaSize);
		map->register_fonts("/usr/share/fonts/truetype", true);
		mapnik::load_map(*map, xmlFile);
		map->set_aspect_fix_mode(mapnik::Map::RESPECT);

		m_maps.push_back(std::move(map));
	}
}

void MapnikLayer::submitRequest(
	const Napi::Promise::Deferred &deferred,
	const Napi::ThreadSafeFunction &callback,
	int x,
	int y,
	int z,
	Layer::StateChangeCallback stateChangeCallback)
{
	auto request = std::make_shared<DetailRequest>(shared_from_this(), deferred, callback, x, y, z, m_root, "png",
		stateChangeCallback);

	m_queue.addRenderRequest(request);
}

MapnikMap MapnikLayer::acquireMap()
{
	std::unique_lock<std::mutex> lock(m_mapMutex);

	if (m_maps.size() == 0)
	{
		m_mapCondition.wait(lock, [this] { return m_maps.size () > 0; });
	}

	auto mapnikMap = std::move(m_maps.back());
	m_maps.pop_back();

	MapnikMap map(*this, std::move(mapnikMap));

	return map;
}

MapnikMap::~MapnikMap()
{
	m_map.m_maps.push_back(std::move(m_mapnikMap));
	m_map.m_mapCondition.notify_all();
}

