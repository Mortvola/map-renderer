#pragma once

#include "RenderRequest.h"
#include "LatLng.h"
#include "ElevationFile.h"
#include "vec3.h"
#include "TextureCoord.h"
#include "Terrain.h"
#include <memory>
#include <vector>

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
  Terrain getElevationTile(double southLat, double westLng, double northLat, double eastLng);

  std::shared_ptr<ElevationFile> loadFile(const LatLng &point);

  void create(const Terrain &ele, double xDimension, double yDimension);

  void createTerrainPoints(
    const Terrain &terrain,
    int numPointsX,
    int numPointsY,
    double xDimension,
    double yDimension
  );

  void createTerrainIndices(
    int numPointsX,
    int numPointsY
  );

  void createTerrainNormals(
    int numPointsX,
    int numPointsY
  );

  void addRoutes(
    double southLat,
    double westLng,
    double northLat,
    double eastLng
  );

  // static constexpr double metersPerPoint {111319.49079327373 / 3600};

  int m_x;
  int m_y;
  int m_dimension;

  std::vector<double> m_points;
  std::vector<int> m_indices;
  std::vector<double> m_normals;

  std::mutex m_fileLoadMutex;
};

void elevationPathSet(const std::string &path);

std::string elevationPathGet();
