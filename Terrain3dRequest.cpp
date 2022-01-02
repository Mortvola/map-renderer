#include "Terrain3dRequest.h"
#include <fstream>

static int terrainVertexStride = 5;

static std::string g_basepath{"./elevations"};

void elevationPathSet(const std::string &path) { g_basepath = path + "/"; }

std::string elevationPathGet() { return g_basepath; }

std::map<std::string, std::shared_ptr<ElevationFile>> files;
uint64_t accessCounter {0};

Terrain3dRequest::Terrain3dRequest (
  const std::shared_ptr<Layer> &layer,
  const Napi::Promise::Deferred &deferred,
  const Napi::ThreadSafeFunction &callback,
  int x,
  int y,
  int z,
  const std::string &fileSystemRoot,
  RenderRequest::StateChangeCallback stateChange)
:
	RenderRequest(layer, RenderRequest::Type::Terrain, deferred, callback, x, y, z, 1, 0, fileSystemRoot, "json", stateChange),
  x(x),
  y(y),
  dimension(z)
{
}

template<class T>
Json::Value vectorToJson(const std::vector<T> &a) {
  Json::Value jsonArray = Json::arrayValue;
  for (const auto &p: a) {
    jsonArray.append(p);
  }

  return jsonArray;
}

template<class T>
Json::Value vector2dToJson(const std::vector<std::vector<T>> &a) {
  Json::Value jsonArray = Json::arrayValue;
  for (const auto &p: a) {
    jsonArray.append(vectorToJson(p));
  }

  return jsonArray;
}

std::vector<std::shared_ptr<Tile>> Terrain3dRequest::render()
{
  std::vector<std::shared_ptr<Tile>> result;

  ele = getElevationTile(x, y, dimension);

  create();

  Json::Value data = Json::objectValue;

  data["points"] = vectorToJson(m_points);
  data["normals"] = vectorToJson(m_normals);
  data["indices"] = vectorToJson(m_indices);
  data["ele"] = vector2dToJson(ele.points);

  Json::FastWriter fastWriter;
  std::string output = fastWriter.write(data);

  result.push_back(m_layer->saveTile(x, y, dimension, output));

  return result;
}

double bilinearInterpolation(double q11, double q12, double q21, double q22,
                             double x, double y) {
  double x2x = 1 - x;
  double y2y = 1 - y;
  return (q11 * x2x * y2y + q12 * x * y2y + q21 * x2x * y + q22 * x * y);
}

Terrain Terrain3dRequest::getElevationTile(int x, int y, int dimension) {
  try {
    auto latLngPerPoint = 1 / 3600.0;

    auto westEdge = x * (dimension - 1);
    auto eastEdge = westEdge + (dimension - 1);
    auto westLng = westEdge * latLngPerPoint - 180;
    auto eastLng = eastEdge * latLngPerPoint - 180;

    auto southEdge = y * (dimension - 1);
    auto northEdge = southEdge + (dimension - 1);
    auto southLat = southEdge * latLngPerPoint - 180;
    auto northLat = northEdge * latLngPerPoint - 180;

    auto latMin = std::floor(southLat);
    auto latMax = std::ceil(northLat);

    auto lngMin = std::floor(westLng);
    auto lngMax = std::ceil(eastLng);

    std::vector<std::vector<uint16_t>> points;
    std::vector<std::vector<double>> centers;

    points.resize(dimension);

    for (auto lat = latMin; lat < latMax; lat++) {
      for (auto lng = lngMin; lng < lngMax; lng++) {

        LatLng nw(lat, lng);
        auto file = loadFile(nw);

        int startX = 0;

        if (lng == lngMin) {
          startX = westEdge % 3600;
        }

        int endX = 3600;

        if (lng == lngMax - 1) {
          endX = eastEdge % 3600;
          if (endX == 0) {
            endX = 3600;
          }
        }

        int startY = southEdge % 3600;
        int yy = 0;

        if (lat != latMin) {
          yy = 3600 - startY;
          startY = 0;
        }

        int endY = 3600;

        if (lat == latMax - 1) {
          endY = northEdge % 3600;
          if (endY == 0) {
            endY = 3600;
          }
        }

        for (auto j = startY; j <= endY; j++) {
          for (auto i = startX; i <= endX; i++) {
            auto e = file->m_buffer[(3600 - j) * 3601 + i];
            points[yy].push_back(swapBytes(e));
          }

          ++yy;
        }
      }
    }

    size_t centersDimension = dimension - 1;
    centers.resize(centersDimension);

    for (size_t j = 0; j < centersDimension; j++) {
      for (size_t i = 0; i < centersDimension; i++) {
        centers[j].push_back(bilinearInterpolation(
            points[j][i], points[j + 1][i], points[j][i + 1],
            points[j + 1][i + 1], 0.5, 0.5));
      }
    }

    Terrain terrain;

    terrain.points = std::move(points);
    terrain.centers = std::move(centers);

    return terrain;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return {};
}


std::tuple<std::string, LatLng> getBaseFileName(const LatLng &point) {
  // Determine file name
  int latInt = std::floor(point.m_lat);
  int lngInt = std::floor(point.m_lng);

  std::stringstream filename;
  filename << std::setfill('0');

  if (point.m_lat >= 0) {
    filename << "N";
    filename << std::setw(2) << latInt;
  } else {
    filename << "S";
    filename << std::setw(2) << -latInt;
  }

  filename << std::setfill('0');

  if (point.m_lng >= 0) {
    filename << "E";
    filename << std::setw(3) << lngInt;
  } else {
    filename << "W";
    filename << std::setw(3) << -lngInt;
  }

  return std::tuple<std::string, LatLng>(filename.str(),
                                         LatLng(latInt, lngInt));
}

std::shared_ptr<ElevationFile> Terrain3dRequest::loadFile(const LatLng &point) {
  std::string filename;
  LatLng latLng;

  std::tie(filename, latLng) = getBaseFileName(point);

  auto fullFileName = elevationPathGet() + filename + ".hgt";

  std::unique_lock<std::mutex> lock(m_fileLoadMutex);

  // See if the file has already been loaded.
  auto iter = files.find(fullFileName);

  if (iter == files.end()) {
    // No, it hasn't been loaded. Load it now.
    auto file = std::ifstream(fullFileName, std::ios_base::binary);

    if (file.is_open()) {
      file.seekg(0, std::ios_base::end);
      auto size = file.tellg();
      file.seekg(0);

      std::cerr << "loading elevation file " << fullFileName << " of size "
                << size << " bytes" << std::endl;

      auto elevationFile = std::make_shared<ElevationFile>();

      elevationFile->m_latLng = latLng;
      elevationFile->m_buffer.resize(size / sizeof(int16_t));
      file.read(reinterpret_cast<char *>(elevationFile->m_buffer.data()), size);

      std::tie(iter, std::ignore) = files.emplace(
          std::pair<std::string, std::shared_ptr<ElevationFile>>(fullFileName, elevationFile));
    } else {
      throw std::runtime_error("Could not read file " + fullFileName);
    }
  }

  accessCounter++;
  iter->second->m_accessCounter = accessCounter;

  if (files.size() > 100) {
    auto oldestAccess = accessCounter;
    auto oldestEntry = files.end();

    for (auto iter = files.begin(); iter != files.end(); ++iter) {
      if (iter->second->m_accessCounter < oldestAccess) {
        oldestEntry = iter;
        oldestAccess = iter->second->m_accessCounter;
      }
    }

    if (oldestEntry != files.end()) {
      files.erase(oldestEntry);
    }
  }

  return iter->second;
}


void Terrain3dRequest::createTerrainIndices(
  int numPointsX,
  int numPointsY
) {
  for (auto i = 0; i < numPointsX - 1; i += 1) {
    m_indices.push_back(i);
    m_indices.push_back(i + 1);
    m_indices.push_back(numPointsX + i * 2 + 1); // center

    m_indices.push_back(i + 1);
    m_indices.push_back(numPointsX + i * 2 + 2);
    m_indices.push_back(numPointsX + i * 2 + 1); // center

    m_indices.push_back(numPointsX + i * 2 + 2);
    m_indices.push_back(numPointsX + i * 2 + 0);
    m_indices.push_back(numPointsX + i * 2 + 1); // center

    m_indices.push_back(numPointsX + i * 2 + 0);
    m_indices.push_back(i);
    m_indices.push_back(numPointsX + i * 2 + 1); // center
  }

  const int firstRowOffset = numPointsX;
  const int numRowPoints = numPointsX * 2 - 1;

  for (auto j = 1; j < numPointsY - 1; j += 1) {
    for (auto i = 0; i < numPointsX - 1; i += 1) {
      m_indices.push_back(firstRowOffset + numRowPoints * (j - 1) + i * 2 + 0);
      m_indices.push_back(firstRowOffset + numRowPoints * (j - 1) + i * 2 + 2);
      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 1);

      m_indices.push_back(firstRowOffset + numRowPoints * (j - 1) + i * 2 + 2);
      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 2);
      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 1);

      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 2);
      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 0);
      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 1);

      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 0);
      m_indices.push_back(firstRowOffset + numRowPoints * (j - 1) + i * 2 + 0);
      m_indices.push_back(firstRowOffset + numRowPoints * (j + 0) + i * 2 + 1);
    }
  }
}

vec3 computeNormal(
  const std::vector<double> &positions,
  const std::vector<int> &indices,
  int index
) {
  vec3 v1 = vec3::fromValues(
    positions[indices[index + 2] * terrainVertexStride + 0]
      - positions[indices[index + 1] * terrainVertexStride + 0],
    positions[indices[index + 2] * terrainVertexStride + 1]
      - positions[indices[index + 1] * terrainVertexStride + 1],
    positions[indices[index + 2] * terrainVertexStride + 2]
    - positions[indices[index + 1] * terrainVertexStride + 2]
  );

  vec3 v2 = vec3::fromValues(
    positions[indices[index] * terrainVertexStride + 0]
      - positions[indices[index + 1] * terrainVertexStride + 0],
    positions[indices[index] * terrainVertexStride + 1]
      - positions[indices[index + 1] * terrainVertexStride + 1],
    positions[indices[index] * terrainVertexStride + 2]
      - positions[indices[index + 1] * terrainVertexStride + 2]
  );

  auto normal = vec3::cross(v1, v2);
  normal = vec3::normalize(normal);

  return normal;
}

void Terrain3dRequest::createTerrainNormals(
  int numPointsX,
  int numPointsY
) {
  // Create a normal for each face
  
  std::vector<vec3> faceNormals;

  auto sumNormals = [&faceNormals](const std::vector<int> &indexes) {
    std::vector<double> vec {0, 0, 0};

    for (std::size_t i = 0; i < indexes.size(); i += 1) {
      vec[0] += faceNormals[indexes[i]][0];
      vec[1] += faceNormals[indexes[i]][1];
      vec[2] += faceNormals[indexes[i]][2];
    }

    auto normal = vec3::fromValues(vec[0], vec[1], vec[2]);
    normal = vec3::normalize(normal);

    return vec3::fromValues(normal[0], normal[1], normal[2]);
  };

  auto concatNormal = [this](vec3 n) {
    m_normals.push_back(n[0]);
    m_normals.push_back(n[1]);
    m_normals.push_back(n[2]);
  };

  for (std::size_t i = 0; i < m_indices.size(); i += 3) {
    faceNormals.push_back(computeNormal(m_points, m_indices, i));
  }

  // Sum the face normals that share a vertex

  // first row

  concatNormal(sumNormals({0, 3}));

  for (auto i = 4; i < (numPointsX - 1) * 4; i += 4) {
    concatNormal(sumNormals({i - 1, i + 3, i + 6, i}));
  }

  concatNormal(sumNormals({(numPointsX - 1) * 4, (numPointsX - 1) * 4 + 1}));

  // interior

  for (auto j = 1; j < numPointsY - 1; j += 1) {
    concatNormal(sumNormals({
      (j - 1) * (numPointsX - 1) * 4 + 3,
      (j - 1) * (numPointsX - 1) * 4 + 2,
      (j + 0) * (numPointsX - 1) * 4 + 0,
      (j + 0) * (numPointsX - 1) * 4 + 3
    }));

    for (auto i = 1; i < numPointsX - 1; i += 1) {
      concatNormal(sumNormals({
        (j - 1) * (numPointsX - 1) * 4 + i * 4 - 1,
        (j - 1) * (numPointsX - 1) * 4 + i * 4 - 2,
        (j - 1) * (numPointsX - 1) * 4 + i * 4 - 3,
        (j - 1) * (numPointsX - 1) * 4 + i * 4 - 4
      }));

      concatNormal(sumNormals({
        (j - 1) * (numPointsX - 1) * 4 + i * 4 - 2,
        (j - 1) * (numPointsX - 1) * 4 + i * 4 - 3,
        (j - 1) * (numPointsX - 1) * 4 + i * 4 + 3,
        (j - 1) * (numPointsX - 1) * 4 + i * 4 + 2,
        (j + 0) * (numPointsX - 1) * 4 + i * 4 - 4,
        (j + 0) * (numPointsX - 1) * 4 + i * 4 - 3,
        (j + 0) * (numPointsX - 1) * 4 + i * 4 + 0,
        (j + 0) * (numPointsX - 1) * 4 + i * 4 + 3
      }));
    }

    concatNormal(sumNormals({
      (j - 1) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 1,
      (j - 1) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 2,
      (j - 1) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 3,
      (j - 1) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 4
    }));

    concatNormal(sumNormals({
      (j - 1) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 2,
      (j - 1) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 3,
      (j + 0) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 4,
      (j + 0) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 3
    }));
  }

  // last row
  concatNormal(sumNormals({
    (numPointsY - 2) * (numPointsX - 1) * 4 + 2,
    (numPointsY - 2) * (numPointsX - 1) * 4 + 3
  }));

  for (auto i = 1; i < numPointsX - 1; i += 1) {
    concatNormal(sumNormals({
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 - 1,
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 - 2,
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 - 3,
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 - 4
    }));

    concatNormal(sumNormals({
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 - 2,
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 - 3,
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 + 3,
      (numPointsY - 2) * (numPointsX - 1) * 4 + i * 4 + 2
    }));
  }

  concatNormal(sumNormals({
    (numPointsY - 2) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 1,
    (numPointsY - 2) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 2,
    (numPointsY - 2) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 3,
    (numPointsY - 2) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 4
  }));

  concatNormal(sumNormals({
    (numPointsY - 2) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 2,
    (numPointsY - 2) * (numPointsX - 1) * 4 + ((numPointsX - 1) * 4) - 3
  }));
}

void Terrain3dRequest::create() {
  int numPointsX = ele.points[0].size();
  int numPointsY = ele.points.size();

  createTerrainPoints(ele, numPointsX, numPointsY);
  createTerrainIndices(numPointsX, numPointsY);
  createTerrainNormals(numPointsX, numPointsY);
}

// eslint-disable-next-line class-methods-use-this
void Terrain3dRequest::createTerrainPoints(
  const Terrain &terrain,
  int numPointsX,
  int numPointsY
) {
  // const { startLatOffset, startLngOffset } = getStartOffset(terrain.sw);

  // Center the tile around the origin.
  // const startLatOffset = -(terrain.ne.lat - terrain.sw.lat) / 2;
  // const startLngOffset = -(terrain.ne.lng - terrain.sw.lng) / 2;

  const double sStep = (terrain.textureNE.s - terrain.textureSW.s) / (numPointsX - 1);
  const double tStep = (terrain.textureNE.t - terrain.textureSW.t) / (numPointsY - 1);

  // we are purposefully using latDistance for both dimensions
  // here to create a square tile (at least for now).
  const double yStep = Terrain3dRequest::metersPerPoint; // this.dimension / (numPointsY - 1);
  const double startYOffset = -((dimension - 1) * Terrain3dRequest::metersPerPoint) / 2;

  const double xStep = Terrain3dRequest::metersPerPoint; // this.dimension / (numPointsX - 1);
  const double startXOffset = -((dimension - 1) * Terrain3dRequest::metersPerPoint) / 2;

  for (auto i = 0; i < numPointsX; i += 1) {
    m_points.push_back(startXOffset + i * xStep);
    m_points.push_back(startYOffset);
    m_points.push_back(terrain.points[0][i]);

    // texture coordinates
    m_points.push_back(terrain.textureSW.s + i * sStep);
    m_points.push_back(terrain.textureSW.t);
  }

  for (auto j = 1; j < numPointsY; j += 1) {
    m_points.push_back(startXOffset);
    m_points.push_back(startYOffset + j * yStep);
    m_points.push_back(terrain.points[j][0]);

    // texture coordinates
    m_points.push_back(terrain.textureSW.s);
    m_points.push_back(terrain.textureSW.t + j * tStep);

    for (auto i = 1; i < numPointsX; i += 1) {
      m_points.push_back(startXOffset + (i - 0.5) * xStep);
      m_points.push_back(startYOffset + (j - 0.5) * yStep);
      m_points.push_back(terrain.centers[j - 1][i - 1]);

      // texture coordinates
      m_points.push_back(terrain.textureSW.s + (i - 0.5) * sStep);
      m_points.push_back(terrain.textureSW.t + (j - 0.5) * tStep);

      m_points.push_back(startXOffset + i * xStep);
      m_points.push_back(startYOffset + j * yStep);
      m_points.push_back(terrain.points[j][i]);

      // texture coordinates
      m_points.push_back(terrain.textureSW.s + i * sStep);
      m_points.push_back(terrain.textureSW.t + j * tStep);
    }
  }
}
