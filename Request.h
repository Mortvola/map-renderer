#pragma once

#include <chrono>
#include <napi.h>

class Request
{
public:
	Request(
		int x,
		int y,
		int z,
		const Napi::Promise::Deferred &deferred);

	Request(const Request &other) = default;

	Napi::Promise::Deferred m_deferred;
	int m_x {0};
	int m_y {0};
	int m_z {0};
};
