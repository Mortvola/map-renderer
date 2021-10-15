#include "TileCache.h"
#include <iostream>
#include <utility>

TileCache g_tileCache;

void TileCache::addTile(const std::shared_ptr<Tile> &tile)
{
  // std::unique_lock<std::mutex> lock(m_cacheMutex);

  auto iter = m_tiles.find(tile->m_filename);

  if (iter == m_tiles.end()) {
    // Tile was not found in the map
    // Add the tile to the front of the MRU
    // and add it to the map
    m_mru.push_front(tile);

    m_tiles.emplace(std::make_pair(tile->m_filename, TileWrapper(tile, m_mru.begin())));

    // If the MRU list is too long then remove the tile at the end of the list
    if (m_mru.size() > 100) {
      auto mruIter = std::prev(m_mru.end());

      if (mruIter != m_mru.end()) {
        auto mapIter = m_tiles.find((*mruIter)->m_filename);

        m_mru.erase(mruIter);
        m_tiles.erase(mapIter);
      }
    }
  }
}

std::shared_ptr<Tile> TileCache::getTile(std::string filename)
{
  // std::unique_lock<std::mutex> lock(m_cacheMutex);

  auto iter = m_tiles.find(filename);

  if (iter != m_tiles.end()) {
    // Tile was found in the map
    // Move it to the front of the MRU 
    // and update the iterator in the map.
    m_mru.erase(iter->second.m_iter);

    m_mru.push_front(iter->second.m_tile);

    iter->second.m_iter = m_mru.begin();

    return iter->second.m_tile;
  }

  return nullptr;
}

