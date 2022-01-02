#include "Terrain3d.h"
#include "ElevationFile.h"
#include "Terrain3dRequest.h"

void Terrain3d::submitRequest(
  const Napi::Promise::Deferred &deferred,
  const Napi::ThreadSafeFunction &callback,
  int x,
  int y,
  int z,
  Layer::StateChangeCallback stateChangeCallback)
{
	auto request = std::make_shared<Terrain3dRequest>(shared_from_this(), deferred, callback, x, y, z, m_root,
		stateChangeCallback);

	m_queue.addRenderRequest(request);
}

Terrain3d::Terrain3d(
		const std::string &stateDir,
		const std::string &name,
    const std::string &elevationDir,
		int workerPoolSize)
:
  Layer(stateDir, name, "json", workerPoolSize)
{
  elevationPathSet(elevationDir);
}
