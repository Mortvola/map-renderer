#include "Utilities.h"
#include <iostream>
#include <array>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <tuple>

void printRenderRange (int x, int y, int z, int metaTiles)
{
	if (metaTiles == 1)
	{
		std::cerr << "(" << z << ", " << y << ", " << x << ")";
	}
	else
	{
		std::cerr << "(" << z << ", " << y * metaTiles << ", " << x * metaTiles << ") to "
				<< "(" << z << ", " << y * metaTiles + metaTiles - 1 << ", " << x * metaTiles + metaTiles - 1 << ")";
	}
}


std::tuple<bool, int> fileExists (const std::string &filename)
{
	struct stat buffer;

	if (stat(filename.c_str (), &buffer) == 0) {
    return std::tuple<bool, int>(true, buffer.st_mtime);
  }

  return std::tuple<bool, int>(false, 0);
}

void fileRemove (const std::string &filename)
{
	unlink(filename.c_str());
}


int createFolder (const std::string &folder)
{
	int result {0};

  bool exists = false;

  std::tie(exists, std::ignore) = fileExists(folder);

	if (!exists)
	{
		result = mkdir(folder.c_str (), 0777);
	}

	return result;
}

void logCreateFolderError (const std::string &folder)
{
	std::cerr << "Unable to create directory " << folder << ": " << strerror (errno) << std::endl;
}

std::string createFolder(const std::string &root, int x, int y, int z)
{
	std::string folder = root + std::to_string(z) + "/";

	auto result = createFolder(root);

	if (result == 0)
	{
		result = createFolder(folder.c_str ());

		if (result == 0)
		{
			std::array<int, 3> name;
			for (size_t i = 0; i < name.size (); i++)
			{
				x >>= 4;
				y >>= 4;

				name[i] = ((x & 0x0f) << 4) | (y & 0x0f);
			}

			for (int i = name.size () - 1; i >= 0; i--)
			{
				folder += std::to_string(name[i]) + "/";
				result = createFolder(folder);

				if (result < 0)
				{
					logCreateFolderError (folder);
				}
			}
		}
		else
		{
			logCreateFolderError (folder);
		}
	}
	else
	{
		logCreateFolderError (root);
	}

	return folder;
}


std::string createFileName (const std::string &root, int x, int y, int z)
{
	auto folder = createFolder (root, x, y, z);

	return folder + "tile_" + std::to_string(z) + "_" + std::to_string(y) + "_" + std::to_string(x) + ".png";
}


void createMD5File (const std::string &filename)
{
	auto result = system (("md5sum " + filename + " | awk '{ printf \"%s\", $1 }' > " + filename + ".md5").c_str ());

	if (result < 0)
	{
		std::cerr << "Error computing md5: " << strerror(errno) << std::endl;
	}
}

std::chrono::milliseconds now()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
}

std::string getEnvVar (const std::string &name, const std::string &defaultValue)
{
	auto value = std::getenv(name.c_str ());

	if (value)
	{
		return value;
	}

	return defaultValue;
}
