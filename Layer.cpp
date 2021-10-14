#include "Layer.h"
#include "Utilities.h"

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
