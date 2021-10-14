#pragma once

#include <string>
#include <chrono>

void printRenderRange (int x, int y, int z, int metaTiles);

bool fileExists (const std::string &filename);

void fileRemove (const std::string &filename);

int createFolder (const std::string &folder);
std::string createFolder(const std::string &root, int x, int y, int z);

void logCreateFolderError (const std::string &folder);

std::string createFileName (const std::string &root, int x, int y, int z);

void createMD5File (const std::string &filename);

std::chrono::milliseconds now();

std::string getEnvVar (const std::string &name, const std::string &defaultValue);
