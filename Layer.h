#pragma once

#include "RenderQueue.h"
#include "TileCache.h"
#include <mapnik/map.hpp>
#include <memory>

class Layer: public std::enable_shared_from_this<Layer>
{
public:
	using StateChangeCallback = std::function<void(const std::shared_ptr<RenderRequest> &)>;

	Layer (
		const std::string &stateDir,
		const std::string &name,
		int workerPoolSize);

	std::string createFileName(int x, int y, int z);

	virtual void submitRequest(
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		StateChangeCallback stateChangeCallback) = 0;

	Napi::Object getQueueInformation(Napi::Env env);

	static const int m_tileSize {256};

	mapnik::box2d<double> m_extent {-20037508.3427892439067364,-20037508.3441838659346104, 20037508.3427892439067364, 20037508.3413946218788624};

  std::shared_ptr<Tile> loadTile(
    int x,
    int y,
    int z);

  std::shared_ptr<Tile> saveTile(
    int x,
    int y,
    int z,
    const std::string &data);

protected:

	std::string m_name;
	std::string m_root;

	RenderQueue m_queue;
};
