// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfmTriangulation.hpp"

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/stl/mapUtils.hpp>

#include <aliceVision/robustEstimation/LORansac.hpp>
#include <aliceVision/robustEstimation/ScoreEvaluator.hpp>
#include <aliceVision/multiview/triangulation/Triangulation.hpp>

#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>

namespace aliceVision {
namespace sfm {

class TriangulationSphericalKernel : public robustEstimation::IRansacKernel<robustEstimation::MatrixModel<Vec4>>
{
public:
    using ModelT = robustEstimation::MatrixModel<Vec4>;
public:  
  /**
   * @brief Constructor.
   * @param[in] _pt2d The feature points, a 2xN matrix.
   * @param[in] projMatrices The N projection matrix for each view.
   */
  TriangulationSphericalKernel(
        const std::vector<Vec2> & observations, 
        const std::vector<Eigen::Matrix4d>& poses, 
        std::vector<std::shared_ptr<camera::IntrinsicBase>> & intrinsics
    )
  : _observations(observations)
  , _poses(poses)
  , _intrinsics(intrinsics)
  {
    for (int id = 0; id < observations.size(); id++)
    {
        const Vec2 & obs = _observations[id];
        std::shared_ptr<camera::IntrinsicBase> camera = _intrinsics[id];

        _lifted.push_back(camera->toUnitSphere(camera->removeDistortion(camera->ima2cam(obs))));
    }
  }

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 2;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 1;
  }

  inline std::size_t getMinimumNbRequiredSamplesLS() const override
  {
    return 2;
  }

  /**
   * @brief Triangulate 2 points.
   * @param[in] samples The index of two points to triangulate.
   * @param[out] models The estimated 3D points.
   */
  void fit(const std::vector<std::size_t>& samples, std::vector<ModelT>& models) const override
  {
    std::vector<Vec3> sampledPts;
    std::vector<Eigen::Matrix4d> sampledMats;
    
    for (int i = 0; i < samples.size(); i++)
    {
        std::size_t idx = samples[i];
        sampledMats.push_back(_poses[idx]);
        sampledPts.push_back(_lifted[idx]);
    }

    _solver.solve(sampledPts, sampledMats, models);
  }

  /**
   * @brief Triangulate N points using the least squared solver.
   * @param[in] inliers The index of the N points to triangulate.
   * @param[out] models The estimated 3D point.
   * @param[in] weights The optional array of weight for each of the N points.
   */
  void fitLS(const std::vector<std::size_t>& inliers,
             std::vector<ModelT>& models,
             const std::vector<double> *weights = nullptr) const override
  {
    std::vector<Vec3> sampledPts;
    std::vector<Eigen::Matrix4d> sampledMats;
    
    for (int i = 0; i < inliers.size(); i++)
    {
        std::size_t idx = inliers[i];
        sampledMats.push_back(_poses[idx]);
        sampledPts.push_back(_lifted[idx]);
    }

    _solver.solve(sampledPts, sampledMats, models, *weights);
  }


  /**
   * @brief Compute the weights..
   * @param[in] model The 3D point for which the weights are computed.
   * @param[in] inliers The array of the indices of the data to be used.
   * @param[out] vec_weights The array of weight of the same size as \p inliers.
   * @param[in] eps An optional threshold to max out the value of the threshold (typically
   * to avoid division by zero or too small numbers).
   */
  void computeWeights(const ModelT & model,
                      const std::vector<std::size_t> &inliers, 
                      std::vector<double>& weights,
                      const double eps = 0.001) const override
  {
    const auto numInliers = inliers.size();
    weights.resize(numInliers);

    for(std::size_t sample = 0; sample < numInliers; ++sample)
    {
      const auto idx = inliers[sample];
      weights[sample] = 1.0 / std::pow(std::max(eps,  error(idx, model)), 2);
    }
  }
  
  /**
   * @brief Error for the i-th view
   * @param[in] sample The index of the view for which the error is computed.
   * @param[in] model The 3D point.
   * @return The estimation error for the given view and 3D point.
   */
  double error(std::size_t sample, const ModelT & model) const override
  {
    Vec4 X = model.getMatrix();
    if (std::abs(X(3)) > 1e-16)
    {
        X = X / X(3);
    }

    Vec2 residual = _intrinsics[sample]->residual(_poses[sample], X, _observations[sample]);

    return residual.norm();
  }

  /**
   * @brief Error for each view.
   * @param[in] model The 3D point.
   * @param[out] vec_errors The vector containing all the estimation errors for every view.
   */
  void errors(const ModelT & model, std::vector<double>& errors) const override
  {
    errors.resize(nbSamples());
    for(Mat::Index i = 0; i < _lifted.size(); ++i)
    {
      errors[i] = error(i, model);
    }
  }

  /**
   * @brief Unnormalize the model. (not used)
   * @param[in,out] model the 3D point.
   */
  void unnormalize(robustEstimation::MatrixModel<Vec4> & model) const override
  {
    
  }

  /**
   * @brief Return the number of view.
   * @return the number of view.
   */
  std::size_t nbSamples() const override
  {
    return _lifted.size();
  }
  
  double logalpha0() const override
  {
    std::runtime_error("Method 'logalpha0()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

  double errorVectorDimension() const override
  {
    std::runtime_error("Method 'errorVectorDimension()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

  double unormalizeError(double val) const override
  {
    std::runtime_error("Method 'unormalizeError()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

  Mat3 normalizer1() const override
  {
    std::runtime_error("Method 'normalizer1()' is not defined for 'NViewsTriangulationLORansac'.");
    return Mat3();
  }

  double thresholdNormalizer() const override
  {
    std::runtime_error("Method 'thresholdNormalizer()' is not defined for 'NViewsTriangulationLORansac'.");
    return 0.0;
  }

private:
  std::vector<Vec3> _lifted;
  const std::vector<Vec2> _observations;
  const std::vector<Eigen::Matrix4d> _poses;
  const std::vector<std::shared_ptr<camera::IntrinsicBase>> _intrinsics;
  multiview::TriangulateNViewsSphericalSolver _solver;
};

bool SfmTriangulation::process(
            const sfmData::SfMData & sfmData,
            const track::TracksMap & tracks,
            const track::TracksPerView & tracksPerView, 
            const feature::FeaturesPerView & featuresPerView,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> &viewIds,
            std::set<IndexT> & evaluatedTracks,
            std::map<IndexT, sfmData::Landmark> & outputLandmarks
        )
{
    evaluatedTracks.clear();

    // Get all tracks id which are visible in views
    std::set<IndexT> viewTracks;
    track::getTracksInImagesFast(viewIds, tracksPerView, viewTracks);

    const std::set<IndexT>& validViews = sfmData.getValidViews();

    // Get a set of all views to consider
    std::set<IndexT> allInterestingViews;
    allInterestingViews.insert(viewIds.begin(), viewIds.end());
    allInterestingViews.insert(validViews.begin(), validViews.end());

//#pragma omp parallel for
    for(int pos = 0; pos < viewTracks.size(); pos++)
    {
        std::set<IndexT>::const_iterator it = viewTracks.begin();
        std::advance(it, pos);

        const std::size_t trackId = *it;
        const track::Track& track = tracks.at(trackId);

        // Get all views observing the current track (Keeping their Id)
        std::set<IndexT> trackViews;
        std::transform(track.featPerView.begin(), track.featPerView.end(),
                       std::inserter(trackViews, trackViews.begin()), stl::RetrieveKey());

        // Intersect with list of interesting views
        std::set<IndexT> trackViewsFiltered;
        std::set_intersection(trackViews.begin(), trackViews.end(), 
                            allInterestingViews.begin(), allInterestingViews.end(), 
                            std::inserter(trackViewsFiltered, trackViewsFiltered.begin()));

        if(trackViewsFiltered.size() < _minObservations)
        {
            continue;
        }

        evaluatedTracks.insert(trackId);

        

        sfmData::Landmark result;
        if (!processTrack(sfmData, track, featuresPerView, randomNumberGenerator, trackViewsFiltered, result))
        {
            continue;
        }

        #pragma omp critical
        {
            outputLandmarks[trackId] = result;
        }
    }

    return true;
}

bool SfmTriangulation::processTrack(
            const sfmData::SfMData & sfmData,
            const track::Track & track,
            const feature::FeaturesPerView & featuresPerView,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            sfmData::Landmark & result
        )
{
    feature::EImageDescriberType descType = track.descType;

    std::vector<Vec2> observations;
    std::vector<std::shared_ptr<camera::IntrinsicBase>> intrinsics;
    std::vector<Eigen::Matrix4d> poses;
    std::vector<IndexT> indexedViewIds;

    for (auto viewId : viewIds)
    {   
        //Retrieve pose and feature coordinates for this observation
        const sfmData::View & view = sfmData.getView(viewId);
        const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsicsharedPtr(view.getIntrinsicId());
        const Eigen::Matrix4d pose = sfmData.getPose(view).getTransform().getHomogeneous();

        std::size_t featureId = track.featPerView.at(viewId);
        const auto & viewFeatures = featuresPerView.getFeaturesPerDesc(viewId);
        const auto & viewFeaturesDesc = viewFeatures.at(descType);
        const auto & pt = viewFeaturesDesc[featureId];

        //Lift the coordinates to metric unit sphere
        const Vec2 coords = pt.coords().cast<double>();

        observations.push_back(coords);
        intrinsics.push_back(intrinsic);
        poses.push_back(pose);
        
        indexedViewIds.push_back(viewId);
    }



    robustEstimation::MatrixModel<Vec4> model;
    std::vector<std::size_t> inliers;
    robustEstimation::ScoreEvaluator<TriangulationSphericalKernel> scorer(8.0);
    TriangulationSphericalKernel kernel(observations, poses, intrinsics);

    model = robustEstimation::LO_RANSAC(kernel, scorer, randomNumberGenerator, &inliers);
    Vec4 X = model.getMatrix();

    Vec3 X_euclidean;
    homogeneousToEuclidean(X, X_euclidean); 

    //Create landmark from result
    result.X = X_euclidean;
    result.descType = track.descType;

    for (const std::size_t & i : inliers)
    {   
        //Inlier to view index
        IndexT viewId = indexedViewIds[i];
        
        sfmData::Observation & o = result.observations[viewId];
        
        //Retrieve observation data
        std::size_t featureId = track.featPerView.at(viewId);
        const auto & viewFeatures = featuresPerView.getFeaturesPerDesc(viewId);
        const auto & viewFeaturesDesc = viewFeatures.at(descType);
        const auto & pt = viewFeaturesDesc[featureId];

        o.id_feat = featureId;
        o.scale = pt.scale();
        o.x = pt.coords().cast<double>();
    }

    return true;
}

} // namespace sfm
} // namespace aliceVision

