// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#pragma once

#include <aliceVision/sfm/pipeline/expanding/LbaPolicy.hpp>

namespace aliceVision {
namespace sfm {

class LbaPolicyConnexity : public LbaPolicy
{
public:
    /**
     * @brief Build the policy using a scene
     * @param sfmData the scene to process
     * @param tracksHandler the tracks for this scene
     * @param views the list of views of interest
    */
    virtual bool build(const sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds);

private:

};

}
}