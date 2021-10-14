#include "RenderQueue.h"
#include "RenderRequest.h"
#include "Utilities.h"
#include <iostream>

RenderQueue::RenderQueue(
	int workerPoolSize)
:
	m_workerPoolSize (workerPoolSize)
{
	// Start render thread
	m_renderThread = std::thread(&RenderQueue::renderThread, this);
}


void RenderQueue::renderThread() {
	for (;;)
	{
		try
		{
			RenderWorker *worker {nullptr};

			{
				std::unique_lock<std::mutex> lock(m_renderQueueMutex);

				std::cerr << "Render Requests Queued: " << m_renderQueue.size () << std::endl;
				std::cerr << "Waiting..." << std::endl;
				m_renderCondition.wait(lock, [this] { return m_renderQueue.size () > 0 && m_rendering.size() < m_workers.size(); });

				std::cerr << "Rendering next item..." << std::endl;

				auto request = m_renderQueue.front ();
				m_renderQueue.pop_front();

				// Find next available worker
				auto iter = std::find_if(m_workers.begin(), m_workers.end(),
					[](const RenderWorker *w)
					{
						return w->m_request == nullptr;
					});

				worker = *iter;
				worker->setRequest(request);

				std::cerr << "Render Requests Queued: " << m_renderQueue.size () << std::endl;
			}

			m_rendering.push_back(worker);
			std::cerr << "Workers Queued: " << m_rendering.size () << std::endl;

			worker->Queue();
		}
		catch (std::exception &e)
		{
			std::cerr << e.what () << std::endl;
		}
	}
}

bool sameMetaTile(
	const std::shared_ptr<RenderRequest> &r1,
	const std::shared_ptr<RenderRequest> &r2)
{
	return (r1->m_x == r2->m_x
		&& r1->m_y == r2->m_y
		&& r1->m_z == r2->m_z);
}


void RenderQueue::addRenderRequest (
	const std::shared_ptr<RenderRequest> &request)
{
	std::cerr << "checking rendering state" << std::endl;

	std::unique_lock<std::mutex> lock(m_renderQueueMutex);

	// Add workers to meet the maximum requested
	for (int i = m_workers.size(); i < m_workerPoolSize; i++) {
		m_workers.push_back(new RenderWorker(request->getEnv(), *this));
	}

	auto iter = std::find_if(m_rendering.begin(), m_rendering.end(),
		[&request](const RenderWorker *worker)
		{
			return sameMetaTile(worker->m_request, request);
		});

	// Is this request the same as the one currently being rendered?
	if (iter != m_rendering.end())
	{
		(*iter)->m_request->addRequestor(request->m_requestors[0]);
	}
	else
	{
		// Determine if this request is already in the queue.
		auto it = std::find_if(m_renderQueue.begin (), m_renderQueue.end (),
			[&request] (const std::shared_ptr<RenderRequest> req)
			{
				return sameMetaTile(req, request);
			});

		if (it != m_renderQueue.end ())
		{
			// Remove the entry from the queue, add the connection
			// and re-add it to the beginning of the queue.
			auto req = *it;

			m_renderQueue.erase(it);

			req->addRequestor(request->m_requestors[0]);

			m_renderQueue.push_front(req);

			std::cerr << "Already queued: ";
			printRenderRange (request->m_x, request->m_y, request->m_z, request->m_metaTiles);
			std::cerr << ", Queued: " << m_renderQueue.size () << std::endl;
		}
		else
		{
			printRenderRange (request->m_requestors[0].m_x, request->m_requestors[0].m_y, request->m_requestors[0].m_z, request->m_metaTiles);

			if (request->m_type == RenderRequest::Type::Terrain)
			{
				std::cerr << "Queued Terrain: ";
			}
			else
			{
				std::cerr << "Queued Detail: ";
			}

			// Put newer requests up front (assumes the client scrolled or zoomed to a new position).
			// Todo: priority should be based on user's connection where all users get equal priority.

			request->setState(RenderRequest::State::Pending);

			m_renderQueue.push_front(request);

			std::cerr << ", Queued: " << m_renderQueue.size () << std::endl;
		}
	}

	m_renderCondition.notify_all();

	std::cerr << "Done adding render request" << std::endl;
}

void RenderQueue::removeWorker(const RenderWorker *worker)
{
	// Remove the worker from the rendering list
	std::unique_lock<std::mutex> lock(m_renderQueueMutex);

	auto iter = std::find(m_rendering.begin(), m_rendering.end(), worker);

	if (iter != m_rendering.end())
	{
		(*iter)->setRequest(nullptr);
		m_rendering.erase(iter);
	}

	m_renderCondition.notify_all();
}

void RenderQueue::requestCompleted (
	const std::shared_ptr<RenderRequest> &request)
{
	std::cerr << "Storing completed request" << std::endl;

	// Add the request to the completed list
	std::unique_lock<std::mutex> lock(m_completedEntriesMutex);
	m_completedEntries.push_front(request);
	if (m_completedEntries.size() > 100)
	{
		m_completedEntries.pop_back();
	}

	std::cerr << "Done storing completed request" << std::endl;
}

Napi::Object RenderQueue::getInformation(Napi::Env env)
{
	auto info = Napi::Object::New(env);

	{
		int i {0};
		std::unique_lock<std::mutex> lock(m_renderQueueMutex);

		auto queued = Napi::Array::New(env, m_rendering.size() + m_renderQueue.size());

		for (const auto &entry: m_rendering) {
			Napi::HandleScope scope(env);
			auto reqObject = entry->m_request->getObject(env);
			queued[i++] = reqObject;
		}

		for (const auto &entry: m_renderQueue) {
			Napi::HandleScope scope(env);
			auto reqObject = entry->getObject(env);
			queued[i++] = reqObject;
		}

		info.Set("queued", queued);
	}

	{
		std::unique_lock<std::mutex> lock(m_completedEntriesMutex);
		int i {0};

		auto completed = Napi::Array::New(env, m_completedEntries.size());

		for (const auto &entry: m_completedEntries) {
			Napi::HandleScope scope(env);
			auto reqObject = entry->getObject(env);
			completed[i++] = reqObject;
		}

		info.Set("completed", completed);
	}

	return info;
}
