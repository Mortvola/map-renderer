#include "Request.h"
#include "Utilities.h"

Request::Request(
	int x,
	int y,
	int z,
	const Napi::Promise::Deferred &deferred)
:
	m_deferred(deferred),
	m_x (x),
	m_y (y),
	m_z (z)
{
}

