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

class SfmResection
{
public:
    SfmResection(size_t maxIterations, double precision) 
    : _maxIterations(maxIterations),
    _precision(precision)
    {

    }

    bool processView(
                const sfmData::SfMData & sfmData,
                const track::TracksMap & tracks,
                const track::TracksPerView & map_tracksPerView, 
                const feature::FeaturesPerView & featuresPerView,
                std::mt19937 &randomNumberGenerator,
                const IndexT viewId,
                Eigen::Matrix4d & updatedPose,
                double & updatedThreshold
            );

private:
    bool internalResection(
            std::shared_ptr<camera::IntrinsicBase> & intrinsic,
            std::mt19937 &randomNumberGenerator,
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations,
            const std::vector<feature::EImageDescriberType> & featureTypes,
            Eigen::Matrix4d & pose,
            std::vector<size_t> & inliers,
            double &errorMax
        );

    bool internalRefinement(
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations,
            const std::vector<size_t> & inliers,
            Eigen::Matrix4d & pose, 
            std::shared_ptr<camera::IntrinsicBase> & intrinsics
        );

private:
    double _precision;
    size_t _maxIterations;
};

} // namespace sfm
} // namespace aliceVision

