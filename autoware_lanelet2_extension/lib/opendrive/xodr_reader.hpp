// Copyright 2026 Autoware Foundation. All rights reserved.
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

#ifndef OPENDRIVE__XODR_READER_HPP_
#define OPENDRIVE__XODR_READER_HPP_

#include "xodr_types.hpp"

#include <lanelet2_io/io_handlers/IoHandler.h>

#include <string>

namespace lanelet::io_handlers::opendrive
{
// Parse the given .xodr file into a POD tree.
// Fatal errors (unopenable / malformed XML, missing <OpenDRIVE> root) throw
// lanelet::ParseError. Non-fatal anomalies are appended to `errors`.
XodrDocument readXodrFile(const std::string & filename, ErrorMessages & errors);

}  // namespace lanelet::io_handlers::opendrive

#endif  // OPENDRIVE__XODR_READER_HPP_
