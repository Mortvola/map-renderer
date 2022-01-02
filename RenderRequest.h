#pragma once

#include "Request.h"
#include "Layer.h"
#include "TileCache.h"
#include <mapnik/map.hpp>
#include <napi.h>
#include <memory>

class Map;

class RenderRequest: public std::enable_shared_from_this<RenderRequest>
{
public:
	using StateChangeCallback = std::function<void(const std::shared_ptr<RenderRequest> &)>;

	enum class Type
	{
		None,
		Detail,
		Terrain,
		Terrain3d,
	};

	enum class State
	{
		Pending,
		Running,
		Success,
		Failure,
	};

	RenderRequest () = default;

	RenderRequest (
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
		const std::string &extension,
		StateChangeCallback stateChange);

	void addRequestor (const Request &request);
	// void setRequestorStates (State state);

	void resolveDeferred(const std::vector<std::shared_ptr<Tile>> &tiles);
	void rejectDeferred();

	Napi::Env getEnv()
	{
		return m_requestors[0].m_deferred.Env();
	}

	mapnik::box2d<double> getBoundingBox () const;

	virtual std::vector<std::shared_ptr<Tile>> render() = 0;

	void setState(State state);

	Napi::Object getObject (Napi::Env env);

	Napi::ThreadSafeFunction m_callback;

	Type m_type {Type::None};
	int m_x {0};
	int m_y {0};
	int m_z {0};
	State m_state {State::Pending};
	std::chrono::milliseconds m_queuedTime {};
	std::chrono::milliseconds m_startTime {};
	std::chrono::milliseconds m_endTime {};
	int m_requestId {0};

	const std::shared_ptr<Layer> m_layer;
	int m_metaTiles {1};
	double m_padding {0.5};

	std::vector<Request> m_requestors;

	std::string m_fileSystemRoot;
	std::string m_extension;

private:

	RenderRequest::StateChangeCallback m_requestStateChange;
};


