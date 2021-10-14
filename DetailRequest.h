#pragma once

#include "RenderRequest.h"

class DetailRequest: public RenderRequest
{
public:

	DetailRequest (
		const std::shared_ptr<Layer> &layer,
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		const std::string &fileSystemRoot,
		RenderRequest::StateChangeCallback stateChange);

	bool render() override;

private:
};
