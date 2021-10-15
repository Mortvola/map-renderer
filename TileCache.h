#pragma once

#include <string>
#include <memory>
#include <map>
#include <list>
#include <mutex>

class Tile
{
public:
  std::string m_filename;
  std::string m_data;
  int m_modificationTime {0};
  std::string m_hash;
};

class TileCache
{
public:
  void addTile(const std::shared_ptr<Tile> &tile);

  std::shared_ptr<Tile> getTile(std::string filename);

private:

  struct TileWrapper
  {
    using MruIterator = std::list<std::shared_ptr<Tile>>::iterator;

    TileWrapper(
      std::shared_ptr<Tile> t,
      MruIterator i
    )
    :
      m_tile (t),
      m_iter (i)
    {
    }

    std::shared_ptr<Tile> m_tile;
    MruIterator m_iter;
  };

  std::map<std::string, TileWrapper> m_tiles;
  std::list<std::shared_ptr<Tile>> m_mru;

	std::mutex m_cacheMutex;
};

extern TileCache g_tileCache;
