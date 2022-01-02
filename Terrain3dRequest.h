#pragma once

#include "RenderRequest.h"
#include "LatLng.h"
#include "ElevationFile.h"
#include "vec3.h"
#include <memory>
#include <vector>

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

struct Terrain
{
  LatLng sw;
  LatLng ne;
  TextureCoord textureSW;
  TextureCoord textureNE;
  std::vector<std::vector<uint16_t>> points;
  std::vector<std::vector<double>> centers;
};

class Terrain3dRequest: public RenderRequest
{
public:

	Terrain3dRequest (
		const std::shared_ptr<Layer> &layer,
		const Napi::Promise::Deferred &deferred,
		const Napi::ThreadSafeFunction &callback,
		int x,
		int y,
		int z,
		const std::string &fileSystemRoot,
		RenderRequest::StateChangeCallback stateChange);

	std::vector<std::shared_ptr<Tile>> render() override;

private:
  Terrain getElevationTile(int x, int y, int dimension);

  const ElevationFile &loadFile(const LatLng &point);

  void create();

  void createTerrainPoints(
    const Terrain &terrain,
    int numPointsX,
    int numPointsY
  );

  void createTerrainIndices(
    int numPointsX,
    int numPointsY
  );

  void createTerrainNormals(
    int numPointsX,
    int numPointsY
  );

  static constexpr double metersPerPoint {111319.49079327373 / 3600};

  int x;
  int y;
  int dimension;

  Terrain ele;
  std::vector<double> m_points;
  std::vector<int> m_indices;
  std::vector<double> m_normals;
};

void elevationPathSet(const std::string &path);

std::string elevationPathGet();
