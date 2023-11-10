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

class SfmTriangulation
{
public:
    SfmTriangulation(size_t minObservations)
    : _minObservations(minObservations)
    {

    }

    bool process(
            const sfmData::SfMData & sfmData,
            const track::TracksMap & tracks,
            const track::TracksPerView & tracksPerView, 
            const feature::FeaturesPerView & featuresPerView,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            std::set<IndexT> & evaluatedTracks,
            std::map<IndexT, sfmData::Landmark> & outputLandmarks
        );

private:
    bool processTrack(
            const sfmData::SfMData & sfmData,
            const track::Track & track,
            const feature::FeaturesPerView & featuresPerView,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            sfmData::Landmark & result
        );  

private:
    const size_t _minObservations;
};

} // namespace sfm
} // namespace aliceVision

