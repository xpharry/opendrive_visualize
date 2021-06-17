#pragma once

#include <iostream>
#include <string>

#include "commonroad/types.hpp"

namespace commonroad {
namespace parser {

enum class XmlInputType : int { FILE, CONTENT };

struct CommonRoadParser {
  static bool Parse(const char *xml,
                    commonroad::CommonRoadData &out_common_road_data,
                    XmlInputType inputType, std::string *out_error = nullptr);
};

} // namespace parser
} // namespace commonroad
