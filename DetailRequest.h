#pragma once

#include "RenderRequest.h"
#include "TileCache.h"
#include <memory>
#include <vector>

class DetailRequest: public RenderRequest
{
public:

	DetailRequest (
		const std::shared_ptr<Layer> &layer,
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		const std::string &fileSystemRoot,
		const std::string &extension,
		RenderRequest::StateChangeCallback stateChange);

	std::vector<std::shared_ptr<Tile>> render() override;

private:
};
