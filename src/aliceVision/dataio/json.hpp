// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <vector>
#include <iostream>
#include <boost/json.hpp>

namespace aliceVision
{

std::vector<boost::json::value> readJsons(std::istream& is, boost::json::error_code& ec);

}