#include "RenderRequest.h"
#include "Utilities.h"
#include <sys/stat.h>
#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>
#include <png++/png.hpp>

static int nextRequestId {0};

RenderRequest::RenderRequest (
	const std::shared_ptr<Layer> &layer,
	Type type,
	const Napi::Promise::Deferred &deferred,
	const Napi::ThreadSafeFunction &callback,
	int x,
	int y,
	int z,
	int metaTiles,
	double padding,
	const std::string fileSystemRoot,
	RenderRequest::StateChangeCallback stateChange)
:
	m_callback(callback),
	m_type (type),
	m_x(x - x % metaTiles),
	m_y(y - y % metaTiles),
	m_z(z),
	m_requestId{++nextRequestId},
	m_layer(layer),
	m_metaTiles (metaTiles),
	m_padding (padding),
	m_fileSystemRoot(fileSystemRoot),
	m_requestStateChange(stateChange)
{
	m_requestors.emplace_back(x, y, z, deferred);
}

void RenderRequest::addRequestor(
	const Request &request)
{
	m_requestors.push_back(request);
}

void RenderRequest::resolveDeferred()
{
	for (const auto &request: m_requestors) {
		m_callback.BlockingCall(
			[this, request](Napi::Env env, Napi::Function jsCallback)
			{
				request.m_deferred.Resolve(
					Napi::String::New(request.m_deferred.Env(),
							createFileName (m_fileSystemRoot, request.m_x, request.m_y, request.m_z)
					)
				);
			});
	}
}

void RenderRequest::rejectDeferred()
{
	for (const auto &request: m_requestors) {
		m_callback.BlockingCall(
			[this, request](Napi::Env env, Napi::Function jsCallback)
			{
				request.m_deferred.Reject(
					Napi::String::New(request.m_deferred.Env(),
						createFileName (m_fileSystemRoot, request.m_x, request.m_y, request.m_z)
					)
				);
			});
	}
}

mapnik::box2d<double> RenderRequest::getBoundingBox () const
{
	auto &extent = m_layer->m_extent;

	int numTilesX = 1 << m_z;
	int numTilesY = 1 << m_z;

	double tileLengthX = ((extent.maxx () - extent.minx ()) / numTilesX);
	double tileLengthY = ((extent.maxy () - extent.miny ()) / numTilesY);
	double metaLengthX = tileLengthX * m_metaTiles + tileLengthX * m_padding * 2;
	double metaLengthY = tileLengthY * m_metaTiles + tileLengthY * m_padding * 2;

	double originX = extent.minx () + tileLengthX * (m_x - m_x % m_metaTiles) - (tileLengthX * m_padding);
	double originY = extent.maxy () - tileLengthY * (m_y - m_y % m_metaTiles) + (tileLengthY * m_padding);

	mapnik::box2d<double> box(
		originX,
		originY,
		originX + metaLengthX,
		originY - metaLengthY);

	return box;
}

void RenderRequest::setState(State state)
{
	m_state = state;

	switch (state)
	{
	case State::Pending:
		m_queuedTime = now();
		break;

	case State::Running:
		m_startTime = now();
		break;

	case State::Success:
	case State::Failure:
		m_endTime = now();
		break;
	}

	m_requestStateChange(shared_from_this());
}


std::string getTypeString(RenderRequest::Type type)
{
	switch (type)
	{
	case RenderRequest::Type::Detail:
		return "detail";
	case RenderRequest::Type::Terrain:
		return "terrain";
	case RenderRequest::Type::None:
		return "none";
	}

	return "unknown";
}

std::string getStateString(RenderRequest::State state)
{
	switch (state)
	{
	case RenderRequest::State::Pending:
		return "pending";
	case RenderRequest::State::Running:
		return "running";
	case RenderRequest::State::Success:
		return "success";
	case RenderRequest::State::Failure:
		return "failure";
	}

	return "unknown";
}

Napi::Object RenderRequest::getObject(Napi::Env env)
{
	auto reqObject = Napi::Object::New(env);

	reqObject.Set("type", getTypeString(m_type));
	reqObject.Set("x", m_x);
	reqObject.Set("y", m_y);
	reqObject.Set("z", m_z);
	reqObject.Set("queuedTime", m_queuedTime.count());
	reqObject.Set("startTime", m_startTime.count());
	reqObject.Set("endTime", m_endTime.count());
	reqObject.Set("requestId", m_requestId);
	reqObject.Set("state", getStateString(m_state));

	return reqObject;
}

