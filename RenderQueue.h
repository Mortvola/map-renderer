#pragma once

#include "RenderWorker.h"
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <napi.h>

class Map;
class RenderWorker;
class RenderRequest;

class RenderQueue
{
public:

	RenderQueue(int workerPoolSize);

	Napi::Object getInformation(Napi::Env env);

	void addRenderRequest (
		const std::shared_ptr<RenderRequest> &request);

	void requestCompleted (
		const std::shared_ptr<RenderRequest> &request);

	void removeWorker(const RenderWorker *worker);

	std::mutex m_renderQueueMutex;
	std::deque<std::shared_ptr<RenderRequest>> m_renderQueue;
	std::vector<RenderWorker *> m_rendering;

	std::mutex m_completedEntriesMutex;
	std::deque<std::shared_ptr<RenderRequest>> m_completedEntries;

	std::thread m_renderThread;
	std::condition_variable m_renderCondition;

private:
	void renderThread();

	int m_workerPoolSize {1};
	std::vector<RenderWorker *> m_workers;
};
