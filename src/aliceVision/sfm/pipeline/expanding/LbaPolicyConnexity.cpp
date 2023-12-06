// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LbaPolicyConnexity.hpp"

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/stl/mapUtils.hpp>

namespace aliceVision {
namespace sfm {


bool LbaPolicyConnexity::build(const sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    
    return true;
}

}
}