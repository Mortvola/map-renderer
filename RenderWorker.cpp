#include "RenderWorker.h"
#include "RenderRequest.h"
#include "Utilities.h"
#include <iostream>

RenderWorker::RenderWorker(
	Napi::Env env,
	RenderQueue &queue)
:
	m_env (env),
	m_queue(queue)
{
	std::cerr << "RenderWorker created" << std::endl;
}

RenderWorker::~RenderWorker() {
	std::cerr << "RenderWorker destroyed" << std::endl;
}

void RenderWorker::setRequest(std::shared_ptr<RenderRequest> request)
{
	m_request = request;
}

void RenderWorker::Queue ()
{
	if (m_thread.joinable())
	{
		m_thread.join();
	}

	m_thread = std::thread(&RenderWorker::Execute, this);
}

void RenderWorker::Execute() {
	try
	{
		m_request->setState(RenderRequest::State::Running);

		auto result = m_request->render();
		if (result.size() == 0)
		{
			m_request->setState(RenderRequest::State::Failure);
			OnError();
		}
		else
		{
			m_request->setState(RenderRequest::State::Success);
			OnOK(result);
		}
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
	}
}

void RenderWorker::OnOK(const std::vector<std::shared_ptr<Tile>> &tiles)
{
	m_request->resolveDeferred(tiles);
	m_queue.requestCompleted(m_request);
	m_queue.removeWorker(this);
}

void RenderWorker::OnError()
{
	m_request->rejectDeferred();
	m_queue.requestCompleted(m_request);
	m_queue.removeWorker(this);
}
