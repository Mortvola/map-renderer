#pragma once

#include <jsoncpp/json/json.h>
#include <iomanip>

class TextureCoord
{
public:

  double s;
  double t;

	operator Json::Value () const
	{
		Json::Value value;

		value["s"] = s;
		value["t"] = t;

		return value;
	}

	friend std::ostream& operator<<(std::ostream &os, const TextureCoord &coord)
	{
		os << std::setprecision(15) <<
			" (" << coord.s <<
			", " << coord.t <<
			") ";

		return os;
	}

private:
};
