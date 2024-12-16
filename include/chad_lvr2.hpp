#pragma once
#include <cstdint>
#include <string_view>
#include "chad/chad.hpp"

void reconstruct(Chad& chad, uint32_t root_addr, std::string_view mesh_name, bool save_grid);