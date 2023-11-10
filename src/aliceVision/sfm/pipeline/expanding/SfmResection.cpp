// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfmResection.hpp"

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/stl/mapUtils.hpp>

#include <aliceVision/robustEstimation/NACRansac.hpp>
#include <aliceVision/numeric/Container.hpp>
#include <aliceVision/multiview/resection/P3PSolver.hpp>
#include <aliceVision/matching/supportEstimation.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>

namespace aliceVision {
namespace sfm {

class KernelResection
{
public:
    using ModelT = Eigen::Matrix4d;

public:
    KernelResection(std::shared_ptr<camera::IntrinsicBase> & camera, 
                    const std::vector<Eigen::Vector3d> & structure, 
                    const std::vector<Eigen::Vector2d> & observations) 
    : _camera(camera), 
    _structure(structure), 
    _observations(observations)
    {
        for (const auto & pt : _observations)
        {
            _liftedObservations.push_back(_camera->toUnitSphere(_camera->cam2ima(_camera->removeDistortion(pt))));
        }
    }

    /**
     * @brief Return the minimum number of required samples for the solver
     * @return minimum number of required samples
     */
    std::size_t getMinimumNbRequiredSamples() const
    {
        return 3;
    }

    /**
     * @brief Return the minimum number of required samples for the solver Ls
     * @return minimum number of required samples
     */
    std::size_t getMinimumNbRequiredSamplesLS() const
    {
        return 3;
    }

    /**
     * @brief Return the maximum number of models for the solver
     * @return maximum number of models
     */
    std::size_t getMaximumNbModels() const
    {
        return 4;
    }

    /**
     * @brief The number of elements in the data.
     * @return the number of elements in the data.
     */
    std::size_t nbSamples() const
    {
        return _structure.size();
    }

    /**
     * @brief Get logalpha0, Alpha0 is used to make the error adaptive to the image size
     * @return logalpha0
     */
    double logalpha0() const
    {
        return log10(M_PI);
    }

    double errorVectorDimension() const
    {
        return 2.0;
    }

    /**
     * @brief This function is called to estimate the model from the minimum number
     * of sample \p minSample (i.e. minimal problem solver).
     * @param[in] samples A vector containing the indices of the data to be used for
     * the minimal estimation.
     * @param[out] models The model(s) estimated by the minimal solver.
     */
    void fit(const std::vector<std::size_t>& samples, std::vector<Eigen::Matrix4d>& models) const
    {
        Mat3 X;
        Mat3 x;

        for (int pos = 0; pos < 3; pos++)
        {
            size_t id = samples[pos];
            X.col(pos) = _structure[id];
            x.col(pos) = _liftedObservations[id];
        }

        _solver.solve(x, X, models);
    }


    /**
     * @brief Function that computes the estimation error for a given model and all the elements.
     * @param[in] model The model to consider.
     * @param[out] vec_errors The vector containing all the estimation errors for every element.
     */
    void errors(const Eigen::Matrix4d & model, std::vector<double>& errors) const 
    {
        for (int idx = 0; idx < _structure.size(); idx++)
        {
            const Vec4 & X = _structure[idx].homogeneous();
            const Vec2 & x = _observations[idx];

            const Vec2 residual = _camera->residual(model, X, x);

            errors[idx] = residual.norm();
        }
    }

private:
    std::shared_ptr<camera::IntrinsicBase> _camera; 
    std::vector<Eigen::Vector3d> _structure;
    std::vector<Eigen::Vector2d> _observations;
    std::vector<Eigen::Vector3d> _liftedObservations;
    multiview::resection::P3PSolver _solver;
};

bool SfmResection::processView(
                        const sfmData::SfMData & sfmData,
                        const track::TracksMap & tracks,
                        const track::TracksPerView & tracksPerView, 
                        const feature::FeaturesPerView & featuresPerView,
                        std::mt19937 &randomNumberGenerator,
                        const IndexT viewId,

                        Eigen::Matrix4d & updatedPose,
                        double & updatedThreshold
                        )
{
    // A. Compute 2D/3D matches
    // A1. list tracks ids used by the view
    const aliceVision::track::TrackIdSet& set_tracksIds = tracksPerView.at(viewId);

    // A2. Each landmark's id is equal to the associated track id
    // Get list of landmarks = get list of reconstructed tracks
    std::set<std::size_t> reconstructedTrackId;
    std::transform(sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(),
                 std::inserter(reconstructedTrackId, reconstructedTrackId.begin()),
                 stl::RetrieveKey());

    // Remove all reconstructed tracks which were not observed in the view to resect.
    // --> Intersection of tracks observed in this view and tracks reconstructed.
    std::set<std::size_t> tracksId;
    std::set_intersection(tracksId.begin(), tracksId.end(),
                        reconstructedTrackId.begin(),
                        reconstructedTrackId.end(),
                        std::inserter(tracksId, tracksId.begin()));


    if (tracksId.size() < 3)
    {
        // If less than 3 points, the resection is theorically impossible.
        // Let ignore this view.
        return false;
    }

    // Associate feature id to each track
    // Feature id is the id of the observation of this track in the current view
    std::vector<track::FeatureId> featuresId;
    track::getFeatureIdInViewPerTrack(tracks, tracksId, viewId, featuresId);


    //Get information about this view
    const std::shared_ptr<sfmData::View> view = sfmData.getViews().at(viewId);
    std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsicsharedPtr(view->getIntrinsicId());

    //Loop over features and tracks to build data needed by resection process
    std::vector<Eigen::Vector3d> structure;
    std::vector<Eigen::Vector2d> observations;
    std::vector<feature::EImageDescriberType> featureTypes;
    auto itFeatures = featuresId.begin();
    auto itTracks = tracksId.begin();
    for (; itFeatures != featuresId.end() && itTracks != tracksId.end(); itFeatures++, itTracks++)
    {
        const feature::EImageDescriberType descType = itFeatures->first;
        const std::size_t trackId = *itTracks;
        const IndexT featureId = itFeatures->second;
        
        const Eigen::Vector3d X = sfmData.getLandmarks().at(trackId).X;
        const Eigen::Vector2d x = featuresPerView.getFeatures(viewId, descType)[featureId].coords().cast<double>();

        structure.push_back(X);
        observations.push_back(x);
        featureTypes.push_back(descType);
    }

    //Compute a first estimation of the pose
    Eigen::Matrix4d pose;
    std::vector<size_t> inliers;
    double errorMax = 0.0;
    if (!internalResection(intrinsic, randomNumberGenerator, structure, observations, featureTypes, pose, inliers, errorMax))
    {
        return false;
    }

    //Refine the pose
    if (!internalRefinement(structure, observations, inliers, pose, intrinsic))
    {
        return false;
    }

    updatedThreshold = errorMax;
    updatedPose = pose;

    return true;
}

bool SfmResection::internalResection(
            std::shared_ptr<camera::IntrinsicBase> & intrinsic,
            std::mt19937 &randomNumberGenerator,
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations,
            const std::vector<feature::EImageDescriberType> & featureTypes,
            Eigen::Matrix4d & pose,
            std::vector<size_t> & inliers,
            double & errorMax
        )
{
    KernelResection kernel(intrinsic, structure, observations);

    Eigen::Matrix4d model;

    inliers.clear();
    auto pairResult = robustEstimation::NACRANSAC(kernel, randomNumberGenerator, inliers, _maxIterations, &model, _precision);

    errorMax = pairResult.first;
    const bool resection = matching::hasStrongSupport(inliers, featureTypes, 3);

    if (resection)
    {
        pose = model;
        return true;
    }

    return false;
}

bool SfmResection::internalRefinement(
        const std::vector<Eigen::Vector3d> & structure,
        const std::vector<Eigen::Vector2d> & observations,
        const std::vector<size_t> & inliers,
        Eigen::Matrix4d & pose, 
        std::shared_ptr<camera::IntrinsicBase> & intrinsics)
{
    // Setup a tiny SfM scene with the corresponding 2D-3D data
    sfmData::SfMData tinyScene;

    // view
    std::shared_ptr<sfmData::View> view = std::make_shared<sfmData::View>("", 0, 0, 0);
    tinyScene.getViews().insert(std::make_pair(0, view));

    // pose
    tinyScene.setPose(*view, sfmData::CameraPose(geometry::Pose3(pose)));

    // Intrinsics
    tinyScene.getIntrinsics().emplace(0, intrinsics);

    const double unknownScale = 0.0;

    // structure data (2D-3D correspondences)
    for(std::size_t i = 0; i < inliers.size(); ++i)
    {
        const std::size_t idx = inliers[i];

        sfmData::Landmark landmark;
        landmark.X = structure[idx];
        landmark.observations[0] = sfmData::Observation(observations[idx], UndefinedIndexT, unknownScale);
        tinyScene.getLandmarks()[i] = std::move(landmark);
    }

    BundleAdjustmentCeres BA;
    BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION;

    const bool success = BA.adjust(tinyScene, refineOptions);
    if(!success)
    {
        return false;
    }

    pose = tinyScene.getPose(*view).getTransform().getHomogeneous();

    return true;
}

} // namespace sfm
} // namespace aliceVision

