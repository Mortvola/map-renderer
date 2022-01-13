#pragma once

#include "Layer.h"

class Terrain3d: public Layer {
public:
	Terrain3d(
		const std::string &stateDir,
		const std::string &name,
    const std::string &elevationDir,
		int workerPoolSize);

	void submitRequest(
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		Layer::StateChangeCallback stateChangeCallback) override;

private:
};
