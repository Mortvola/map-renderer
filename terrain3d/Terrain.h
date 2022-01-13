#pragma once

struct Terrain
{
  LatLng sw;
  LatLng ne;
  TextureCoord textureSW;
  TextureCoord textureNE;
  std::vector<std::vector<uint16_t>> points;
  std::vector<std::vector<double>> centers;
};
