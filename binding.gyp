{
  'variables': {
      'pkg-config': 'pkg-config',
      'mapnik-config': 'mapnik-config'
  },
  "targets": [{
    "target_name": "rendermap",
    "cflags!": [
      "-fno-exceptions",
    ],
    "cflags_cc": [
      '<!@(<(mapnik-config) --cflags --cxxflags)',
      '-std=c++17',
      '-Wno-shadow',
      '-O2'
    ],
    'cflags_cc!': [
      '-fno-rtti',
      "-fno-exceptions",
    ],
    "sources": [
        "rendermap.cpp",
        "RenderWorker.cpp",
        "RenderRequest.cpp",
        "TerrainRequest.cpp",
        "DetailRequest.cpp",
        "RenderQueue.cpp",
        "Request.cpp",
        "Utilities.cpp",
        "Layer.cpp",
        "MapnikLayer.cpp",
        "GeoTiffLayer.cpp",
        "TileCache.cpp",
        "terrain3d/Terrain3d.cpp",
        "terrain3d/Terrain3dRequest.cpp",
        "terrain3d/vec3.cpp",
    ],
    'include_dirs': [
        "<!@(node -p \"require('node-addon-api').include\")",
      ".",
    ],
    "libraries": [
        '<!@(<(mapnik-config) --libs --dep-libs)',
        '-ljsoncpp',
        '-lgdal'
      ],        
    'dependencies': [
        "<!(node -p \"require('node-addon-api').gyp\")"
    ],
    'defines': [
      'NAPI_DISABLE_CPP_EXCEPTIONS',
      ]
  }]
}
