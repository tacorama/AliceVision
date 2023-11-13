// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ExpansionProcess.hpp"

#include <aliceVision/sfm/pipeline/expanding/SfmTriangulation.hpp>

namespace aliceVision {
namespace sfm {

bool ExpansionProcess::process(sfmData::SfMData & sfmData, const track::TracksMap& tracks)
{
    //Prepare existing data
    prepareExisting(sfmData, tracks);

    return true;
}

void ExpansionProcess::prepareExisting(sfmData::SfMData & sfmData, const track::TracksMap& tracks)
{
    //Prepare existing data
    remapExistingLandmarks(sfmData, tracks);

    // If there are some poses existing
    // We want to make sure everything is on par with requirements
    if (sfmData.getPoses().empty() == false)
    {
        upgradeSfm(sfmData, tracks);
    }

}

void ExpansionProcess::upgradeSfm(sfmData::SfMData & sfmData, const track::TracksMap& tracks)
{
    SfmTriangulation triangulation(_minTriangulationObservations);
    
}

void ExpansionProcess::remapExistingLandmarks(sfmData::SfMData & sfmData, const track::TracksMap& tracks)
{
    // get unmap landmarks
    sfmData::Landmarks landmarks;

    // clear sfmData structure and store them locally
    std::swap(landmarks, sfmData.getLandmarks());

    // builds landmarks temporary comparison structure
    // ObsKey <ViewId, FeatId, decType>
    // ObsToLandmark <ObsKey, LandmarkId>
    using ObsKey = std::tuple<IndexT, IndexT, feature::EImageDescriberType>;
    using ObsToLandmark = std::map<ObsKey, IndexT>;

    ObsToLandmark obsToLandmark;
    for (const auto& landmarkPair : landmarks)
    {
        const IndexT landmarkId = landmarkPair.first;
        if (landmarkPair.second.observations.size() == 0)
        {
            continue;
        }

        const IndexT firstViewId = landmarkPair.second.observations.begin()->first;
        const IndexT firstFeatureId = landmarkPair.second.observations.begin()->second.id_feat;
        const feature::EImageDescriberType descType = landmarkPair.second.descType;

        obsToLandmark.emplace(ObsKey(firstViewId, firstFeatureId, descType), landmarkId);
    }


    // For each track
    for (const auto & trackPair : tracks)
    {
        const IndexT trackId = trackPair.first;
        const track::Track& track = trackPair.second;

        //For each feature in track
        for (const auto& featView : track.featPerView)
        {
            ObsKey key(featView.first, featView.second.featureId, track.descType);

            //We assume one feature is associated to only one track
            const ObsToLandmark::const_iterator it = obsToLandmark.find(key);

            if (it == obsToLandmark.end())
            {
                continue;
            }

            auto landmarkPair = landmarks.find(it->second);
            landmarks.erase(landmarkPair->first);

            // re-insert the landmark with the new id
            sfmData.getLandmarks().emplace(trackId, landmarkPair->second);
        }
    }

    if (landmarks.size() > 0)
    {
        ALICEVISION_LOG_INFO("Not all existing landmarks have been remapped");
    }
}

} // namespace sfm
} // namespace aliceVision

