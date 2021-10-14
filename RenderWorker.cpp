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
		std::cerr << "doing stuff asynchronously" << std::endl;

		m_request->setState(RenderRequest::State::Running);

		auto result = m_request->render();
		if (!result)
		{
			m_request->setState(RenderRequest::State::Failure);
			OnError();
		}
		else
		{
			m_request->setState(RenderRequest::State::Success);
			OnOK();
		}

		std::cerr << "done doing stuff asynchronously" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
	}
}

void RenderWorker::OnOK() {
	std::cerr << "In OnOK. Resolving promise" << std::endl;
	m_request->resolveDeferred();
	m_queue.requestCompleted(m_request);
	m_queue.removeWorker(this);
	std::cerr << "Done resolving promise" << std::endl;
}

void RenderWorker::OnError()
{
	std::cerr << "In OnError. Rejecting promise" << std::endl;
	m_request->rejectDeferred();
	m_queue.requestCompleted(m_request);
	m_queue.removeWorker(this);
	std::cerr << "Done rejecting promise" << std::endl;
}
