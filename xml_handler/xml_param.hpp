// Copyright 2023 Komáromi Sándor
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef  XML__XML_PARAM_H_
#define  XML__XML_PARAM_H_

#include <variant>

#include "xml_string.hpp"

namespace xml
{
enum class XMLType : size_t
{
  BOOL = 0,
  LONG = 1,
  DOUBLE = 2,
  STRING = 3
};


class XMLParam
{
public:
  XMLType param_type_;
  std::variant<bool, long, double, XMLString> param_;
  XMLParam() = default;
  XMLParam(XMLType type)
  : param_type_(type) {}
  size_t ParamSprint(char * & buffer_it, char * const buffer, const size_t buffer_size);

  template<typename T>
  T GetParam() const
  {
    if constexpr (std::is_same<T, bool>::value || std::is_same<T, long>::value ||
      std::is_same<T, double>::value || std::is_same<T, XMLString>::value)
    {
      return std::get<T>(param_);
    }
    throw std::logic_error("Parameter type not supported");
  }
};

}
#endif  // XML__XML_PARAM_H_
