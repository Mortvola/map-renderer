#pragma once

#include "RenderRequest.h"
#include "Layer.h"
#include "Utilities.h"
#include <napi.h>
#include <memory>
#include <map>

class rendermap: public Napi::ObjectWrap<rendermap>
{
public:

	static Napi::Object Init(Napi::Env env, Napi::Object exports);

	rendermap (const Napi::CallbackInfo &info);

private:

	struct Configuration
	{
		std::string mapnikFile {"./mapnik.xml"};
		std::string terrainDir {"."};
		int metaTiles {1};
		int mapnikPoolSize {1};
	};

	static Napi::FunctionReference constructor;

	Napi::Value render(const Napi::CallbackInfo &info);

	enum class RenderStatus {
		Queued,
		Rendered,
		Error,
	};

	std::tuple<RenderStatus, std::string> render (
		const std::shared_ptr<Layer> &layer,
		const Napi::Promise::Deferred &deferred,
		int x,
		int y,
		int z,
		bool update);

	Napi::Value getRequests(const Napi::CallbackInfo &info);
	Napi::Object getQueueInformation(Napi::Env env);

	Napi::Promise processRequest (
		const Napi::Object &request,
		Napi::Env env);

	void requestStateChange(
		const std::shared_ptr<RenderRequest> &request);

	std::map<std::string, std::shared_ptr<Layer>> m_layers;

	Napi::ThreadSafeFunction m_callbackFunction;
	Napi::ThreadSafeFunction m_requestStateChangeCallback {};
};

