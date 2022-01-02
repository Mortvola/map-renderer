#pragma once

#include <string>
#include <chrono>

void printRenderRange (int x, int y, int z, int metaTiles);

std::tuple<bool, int> fileExists (const std::string &filename);

void fileRemove (const std::string &filename);

int createFolder (const std::string &folder);
std::string createFolder(const std::string &root, int x, int y, int z);

void logCreateFolderError (const std::string &folder);

std::string createFileName (const std::string &root, int x, int y, int z, const std::string &extension);

void createMD5File (const std::string &filename);

std::chrono::milliseconds now();

std::string getEnvVar (const std::string &name, const std::string &defaultValue);
