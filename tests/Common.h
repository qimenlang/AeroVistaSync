#pragma once

#include "SyncTestHelpers.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

class TempConfigFile
{
public:
    explicit TempConfigFile(const std::string& jsonBody)
    {
        _path = (std::filesystem::temp_directory_path() /
                 ("ave_engine_cfg_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json"))
                    .string();
        std::ofstream out(_path, std::ios::binary);
        out << jsonBody;
    }

    ~TempConfigFile()
    {
        std::error_code ec;
        std::filesystem::remove(_path, ec);
    }

    const std::string& path() const { return _path; }

private:
    std::string _path;
};
