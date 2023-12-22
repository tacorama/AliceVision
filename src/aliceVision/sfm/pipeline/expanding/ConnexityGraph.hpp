// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <lemon/list_graph.h>

namespace aliceVision {
namespace sfm {

class ConnexityGraph
{
public:
    bool build(const sfmData::SfMData & sfmData, const std::set<IndexT> & viewsOfInterest);

private:
    size_t _minLinksPerView = 10;
    size_t _minCardinality = 50;
};

}
}