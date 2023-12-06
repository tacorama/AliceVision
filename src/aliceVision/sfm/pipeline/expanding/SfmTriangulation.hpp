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
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            std::set<IndexT> & evaluatedTracks,
            std::map<IndexT, sfmData::Landmark> & outputLandmarks
        );

    static bool checkChierality(const sfmData::SfMData & sfmData, const sfmData::Landmark & landmark);
    static double getMaximalAngle(const sfmData::SfMData & sfmData, const sfmData::Landmark & landmark);

private:
    bool processTrack(
            const sfmData::SfMData & sfmData,
            const track::Track & track,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            sfmData::Landmark & result
        );  

private:
    const size_t _minObservations;
};

} // namespace sfm
} // namespace aliceVision

