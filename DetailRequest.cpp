#include "DetailRequest.h"
#include "MapnikLayer.h"
#include "TileCache.h"
#include "Utilities.h"
#include <sys/stat.h>
#include <fstream>
#include <openssl/md5.h>
#include <sstream>
#include <iomanip>
#include <memory>

DetailRequest::DetailRequest (
	const std::shared_ptr<Layer> &layer,
	const Napi::Promise::Deferred &deferred,
	const Napi::ThreadSafeFunction &callback,
	int x,
	int y,
	int z,
	const std::string &fileSystemRoot,
	RenderRequest::StateChangeCallback stateChange)
:
	RenderRequest(
		layer,
		RenderRequest::Type::Detail,
		deferred,
		callback,
		x,
		y,
		z,
		static_cast<MapnikLayer *>(layer.get())->m_metaTiles,
		0.5,
		fileSystemRoot,
		stateChange)
{
}

std::vector<std::shared_ptr<Tile>> DetailRequest::render ()
{
  std::vector<std::shared_ptr<Tile>> result;
	std::cerr << m_layer->m_extent << std::endl;

	auto box = getBoundingBox ();

	std::cerr << box << std::endl;

	auto mapnikLayer = static_cast<MapnikLayer *>(m_layer.get());
	auto map = mapnikLayer->acquireMap();

	map.zoom_to_box(box);

	mapnik::image_rgba8 image(mapnikLayer->m_metaSize, mapnikLayer->m_metaSize);
	mapnik::agg_renderer<mapnik::image_rgba8> ren(map, image);
	ren.apply();

	for (int y = 0; y < m_metaTiles; y++)
	{
		for (int x = 0; x < m_metaTiles; x++)
		{
			mapnik::image_view<mapnik::image_rgba8> iv(
					m_padding * m_layer->m_tileSize + x * m_layer->m_tileSize,
					m_padding * m_layer->m_tileSize + y * m_layer->m_tileSize,
					m_layer->m_tileSize,
					m_layer->m_tileSize,
					image);
			struct mapnik::image_view_any vw(iv);

      result.push_back(m_layer->saveTile(m_x + x, m_y + y, m_z, mapnik::save_to_string(vw, "png")));
		}
	}

	return result;
}
