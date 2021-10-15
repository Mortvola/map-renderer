#include "Layer.h"
#include "Utilities.h"
#include <tuple>
#include <fstream>
#include <openssl/md5.h>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>

Layer::Layer(const std::string &stateDir, const std::string &name, int workerPoolSize)
:
	m_name(name),
	m_root(stateDir + "/" + name + "/"),
	m_queue(workerPoolSize)
{
	createFolder(m_root);
}

std::string Layer::createFileName(int x, int y, int z)
{
	return ::createFileName(m_root, x, y, z);
}

Napi::Object Layer::getQueueInformation(Napi::Env env)
{
	return m_queue.getInformation(env);
}

std::string createHash(const std::string &data)
{
  unsigned char result[MD5_DIGEST_LENGTH];
  MD5(reinterpret_cast<const unsigned char*>(data.c_str()), data.length(), result);

  std::stringstream ss;
  for (auto c: result)
  {
    ss << std::setw(2) << std::setfill('0') << static_cast<int>(c);
  }

  return ss.str();
}

std::shared_ptr<Tile> Layer::loadTile(
  int x,
  int y,
  int z)
{
   auto tile = std::make_shared<Tile>();

  tile->m_filename = createFileName(x, y, z);

  {
    std::ifstream file(tile->m_filename, std::ios::binary);
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    tile->m_data.resize(size);
    file.seekg(0);
    file.read(&tile->m_data[0], size);
  }

  struct stat buffer;
  if (stat(tile->m_filename.c_str (), &buffer) == 0)
  {
    tile->m_modificationTime = buffer.st_mtime;
  }

  tile->m_hash = createHash(tile->m_data);

  g_tileCache.addTile(tile);

  return tile;
}

std::shared_ptr<Tile> Layer::saveTile(
  int x,
  int y,
  int z,
  const std::string &data)
{
  auto tile = std::make_shared<Tile>();

  tile->m_filename = createFileName(x, y, z);

  tile->m_data = data;

  {
    std::ofstream file(tile->m_filename, std::ios::binary);
    file.write(tile->m_data.c_str(), tile->m_data.length());
  }

  chmod(tile->m_filename.c_str (), 0666);

  struct stat buffer;
  if (stat(tile->m_filename.c_str (), &buffer) == 0)
  {
    tile->m_modificationTime = buffer.st_mtime;
  }

  tile->m_hash = createHash(tile->m_data);

  {
    std::ofstream file(tile->m_filename + ".md5", std::ios::binary);
    file.write(tile->m_hash.c_str(), tile->m_hash.length());
  }

  g_tileCache.addTile(tile);

  return tile;
}
