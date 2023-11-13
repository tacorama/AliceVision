// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>

namespace aliceVision {
namespace sfm {

class ExpansionProcess{
public:
    bool process(sfmData::SfMData & sfmData, 
                const track::TracksMap& tracks);

private:

    /**
     * @brief Process sfmData if something exists inside (previous sfm)
     * @param[in] sfmData the object to update
     * @param[in] tracks all tracks of the scene as a map {trackId, track}
     */
    void prepareExisting(sfmData::SfMData & sfmData, const track::TracksMap& tracks);

    /**
     * @brief Remap the sfmData landmarks to id compatible with trackmap
     * @param[in] sfmData the object to update
     * @param[in] tracks all tracks of the scene as a map {trackId, track}
     */
    void remapExistingLandmarks(sfmData::SfMData & sfmData, const track::TracksMap& tracks);

    /**
     * @brief Try to upgrade sfm with new landmarks
     * @param[in] sfmData the object to update
     * @param[in] tracks all tracks of the scene as a map {trackId, track}
     */
    void upgradeSfm(sfmData::SfMData & sfmData, const track::TracksMap& tracks);

private:
    
    const int _minTriangulationObservations = 2;
};

} // namespace sfm
} // namespace aliceVision

