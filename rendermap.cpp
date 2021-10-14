/*
 * mapRender.cpp
 *
 *  Created on: Dec 30, 2019
 *		Author: richard
 */

#include "rendermap.h"
#include "RenderWorker.h"
#include "DetailRequest.h"
#include "TerrainRequest.h"
#include "MapnikLayer.h"
#include "GeoTiffLayer.h"
#include <cmath>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <jsoncpp/json/json.h>
#include <csignal>
#include <boost/filesystem.hpp>
#include <regex>

Napi::FunctionReference rendermap::constructor;

Napi::Object InitAll(Napi::Env env, Napi::Object exports)
{
	return rendermap::Init(env, exports);
}

NODE_API_MODULE(NODE_GYP_MODULE_NAME, InitAll)

rendermap::rendermap (const Napi::CallbackInfo &info)
:
	Napi::ObjectWrap<rendermap>(info)
{
	Napi::Env env = info.Env();

	m_callbackFunction = Napi::ThreadSafeFunction::New(
		env,
		{},
		"callback",
		0,
		1
	);

	std::cerr << "number of parameters: " << info.Length() << std::endl;

	if (info.Length() == 1 && info[0].IsObject())
	{
		auto initObject = info[0].As<Napi::Object>();

		if (initObject.Has("requestStateChange"))
		{
			std::cerr << "Has requestStateChange" << std::endl;

			auto requestStateChangeCallback = initObject.Get("requestStateChange");

			m_requestStateChangeCallback = Napi::ThreadSafeFunction::New(
				env,
				requestStateChangeCallback.As<Napi::Function>(),
				"requestStateChange",
				0,
				1);
		}

		auto stateDir = getEnvVar ("STATE_DIRECTORY", "/var/lib/mapRender");

		if (initObject.Has("layers") && initObject.Get("layers").IsObject())
		{
			auto layers = initObject.Get("layers").As<Napi::Object>();
			auto layerNames = layers.GetPropertyNames();

			for (uint32_t i = 0; i < layerNames.Length(); i++)
			{
				std::string layerName = static_cast<Napi::Value>(layerNames[i]).As<Napi::String>();

				auto layer = layers.Get(layerName).As<Napi::Object>();

				std::string type;

				if (layer.Has("type") && layer.Get("type").IsString())
				{
					type = layer.Get("type").As<Napi::String>();
				}

				if (type == "mapnik")
				{
					std::string mapnikFile {"./mapnik.xml"};
					std::string terrainDir {"."};
					int metaTiles {1};
					int mapnikPoolSize {1};

					if (layer.Has("file") && layer.Get("file").IsString())
					{
						mapnikFile = layer.Get("file").As<Napi::String>();
					}

					if (layer.Has("mapnikPoolSize") && layer.Get("mapnikPoolSize").IsNumber())
					{
						mapnikPoolSize = layer.Get("mapnikPoolSize").As<Napi::Number>();
					}

					if (layer.Has("metaTiles") && layer.Get("metaTiles").IsNumber())
					{
						metaTiles = layer.Get("metaTiles").As<Napi::Number>();
					}

					std::cerr << "mapnik file: " << mapnikFile << std::endl;

					auto layer = std::make_unique<MapnikLayer>(
						stateDir,
						layerName,
						mapnikFile,
						metaTiles,
						mapnikPoolSize,
						4);

					m_layers[layerName] = std::move(layer);
				}
				else if (type == "geotiff")
				{
					std::string terrainDir {"."};

					if (layer.Has("directory") && layer.Get("directory").IsString())
					{
						terrainDir = layer.Get("directory").As<Napi::String>();
					}

					auto layer = std::make_unique<GeoTiffLayer>(
						stateDir,
						layerName,
						terrainDir,
						1);

					m_layers[layerName] = std::move(layer);
				}
			}
		}
	}
}

std::tuple<double, double> latLngToWebMercator (double lng, double lat)
{
	static const double degreesToRadians {0.017453292519943295};
	static const double earthRadius {6378137.0};

	double x = earthRadius * lng * degreesToRadians;
	double radY = lat * degreesToRadians;
	double y = (earthRadius / 2) * std::log((1.0 + std::sin(radY)) / (1.0 - std::sin(radY)));

	return std::tuple<double, double> (x, y);
}


std::tuple<rendermap::RenderStatus, std::string> rendermap::render(
	const std::shared_ptr<Layer> &layer,
	const Napi::Promise::Deferred &deferred,
	int x,
	int y,
	int z,
	bool update)
{
	try
	{
		auto filename = layer->createFileName (x, y, z);

		std::cerr << "Requested " << (update ? "(with update) " : "") << filename << std::endl;

		if (update || !fileExists(filename))
		{
			fileRemove(filename);

			// Add tile to render to the render queue
			layer->submitRequest(deferred, m_callbackFunction, x, y, z,
				[this](const std::shared_ptr<RenderRequest> &request)
				{
					requestStateChange(request);
				});

			return std::tuple<RenderStatus, std::string>(RenderStatus::Queued, {});
		}

		if (!fileExists (filename + ".md5"))
		{
			createMD5File (filename);
		}

		return std::tuple<RenderStatus, std::string>(RenderStatus::Rendered, filename);
	}
	catch (std::exception &e)
	{
		std::cerr << e.what () << ", tile: " << x << ", " << y << ", " << z << std::endl;
	}

	return std::tuple<RenderStatus, std::string>(RenderStatus::Error, {});
}


Napi::Promise rendermap::processRequest (
	const Napi::Object &request,
	Napi::Env env)
{
	Napi::Promise::Deferred deferred = Napi::Promise::Deferred::New(env);

	std::string type = request["type"].ToString();
	int32_t x = request["x"].ToNumber();
	int32_t y = request["y"].ToNumber();
	int32_t z = request["z"].ToNumber();

	bool update = false;
	if (request.Has("update"))
	{
		update = request.Get("update").As<Napi::Boolean>();
	}

	RenderStatus result;
	std::string fileName;

	auto iter = m_layers.find(type);

	if (iter != m_layers.end())
	{
		auto &layer = iter->second;

		std::tie(result, fileName) = render(layer, deferred, x, y, z, update);
	}
	else
	{
		std::cerr << "Unknown type: " << type << std::endl;
		result = RenderStatus::Error;
	}

	switch (result) {
	case RenderStatus::Rendered:
		deferred.Resolve(Napi::String::New(deferred.Env(), fileName));
		break;
	case RenderStatus::Error:
		deferred.Reject(Napi::String::New(deferred.Env(), fileName));
		break;
	case RenderStatus::Queued:
		break;
	}

	return deferred.Promise();
}

void rendermap::requestStateChange(
	const std::shared_ptr<RenderRequest> &request)
{
	std::cerr << m_requestStateChangeCallback << std::endl;

	auto result = m_requestStateChangeCallback.Acquire();

	if (result == napi_ok)
	{
		m_requestStateChangeCallback.BlockingCall(
			[request](Napi::Env env, Napi::Function jsCallback)
			{
				jsCallback.Call({ request->getObject(env) });
			}
		);

		m_requestStateChangeCallback.Release();
	}
}

Napi::Object rendermap::getQueueInformation (Napi::Env env)
{
	auto queueInfo = Napi::Object::New(env);

	for (const auto &layer: m_layers)
	{
		auto info  = layer.second->getQueueInformation(env);

		queueInfo.Set(layer.first, info);
	}

	return queueInfo;
}

Napi::Value rendermap::getRequests(const Napi::CallbackInfo &info)
{
	return getQueueInformation(info.Env());
}

Napi::Value rendermap::render(const Napi::CallbackInfo &info) {
	Napi::Env env = info.Env();

	if (info.Length() != 1 || !info[0].IsObject()) {
		Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
	}

	auto request = info[0].ToObject();

	return processRequest(request, env);
}

Napi::Object rendermap::Init(Napi::Env env, Napi::Object exports)
{
	Napi::HandleScope scope(env);

	Napi::Function func = DefineClass(env, "Renderer", {
		InstanceMethod("render", &rendermap::render),
		InstanceMethod("getRequests", &rendermap::getRequests),
	});

	constructor = Napi::Persistent(func);
	constructor.SuppressDestruct();

	exports.Set("Renderer", func);

	return exports;
}
