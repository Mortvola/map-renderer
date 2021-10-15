#pragma once

#include "TileCache.h"
#include <napi.h>
#include <thread>
#include <memory>
#include <vector>

class RenderQueue;
class RenderRequest;

class RenderWorker
{
public:

	RenderWorker(
		Napi::Env env,
		RenderQueue &queue);

	~RenderWorker();

	void setRequest(std::shared_ptr<RenderRequest> request);

	void Queue ();
	void Execute();

	void OnOK(const std::vector<std::shared_ptr<Tile>> &tile);
	void OnError();

	std::thread m_thread;
	Napi::Env m_env;
	RenderQueue &m_queue;
	std::shared_ptr<RenderRequest> m_request;

private:
};

