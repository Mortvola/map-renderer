#pragma once

#include "RenderRequest.h"

class TerrainRequest: public RenderRequest
{
public:

	TerrainRequest (
		const std::shared_ptr<Layer> &layer,
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		const std::string &fileSystemRoot,
		RenderRequest::StateChangeCallback stateChange);

	bool render() override;

private:
};
