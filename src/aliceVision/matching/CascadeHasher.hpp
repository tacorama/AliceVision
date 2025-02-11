// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/feature/metric.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/stl/DynamicBitset.hpp>

#include <iostream>
#include <random>
#include <cmath>

namespace aliceVision {
namespace matching {

struct HashedDescription
{
    // Hash code generated by the primary hashing function.
    stl::dynamic_bitset hash_code;

    // Each bucket_ids[x] = y means the descriptor belongs to bucket y in bucket
    // group x.
    std::vector<uint16_t> bucket_ids;
};

struct HashedDescriptions
{
    // The hash information.
    std::vector<HashedDescription> hashed_desc;

    typedef std::vector<int> Bucket;
    // buckets[bucket_group][bucket_id] = bucket (container of description ids).
    std::vector<std::vector<Bucket>> buckets;
};

/**
 * This hasher will hash descriptors with a two-step hashing system:
 * 1. it generates a hash code,
 * 2. it determines which buckets the descriptors belong to.
 * Retrieval step is fast since:
 *  - only descriptors in the same bucket are likely to be good matches.
 *  - 1. a hamming distance is used for fast neighbor candidate retrieval
 *  - 2. the L2 distance is computed only a reduced selection of approximate neighbor
 *
 * Implementation is based on the paper:
 * [1] "Fast and Accurate Image Matching with Cascade Hashing for 3D Reconstruction"
 * Authors: Jian Cheng, Cong Leng, Jiaxiang Wu, Hainan Cui, Hanqing Lu.
 * Date: 2014.
 * Conference: CVPR.
 *
 * This implementation is based on the Theia library implementation from Chris Sweeney.
 * Update compare to the initial paper [1] and initial author code:
 * - hashing projection is made by using Eigen to use vectorization
 * - replace the BoxMuller random number generation by C++ 11 random number generation
 * - this implementation can support various descriptor length and internal type
 *   SIFT, SURF, ... all scalar based descriptor
 */
class CascadeHasher
{
  private:
    // The number of bucket bits.
    int nb_bits_per_bucket_;
    // The number of dimensions of the Hash code.
    int nb_hash_code_;
    // The number of bucket groups.
    int nb_bucket_groups_;
    // The number of buckets in each group.
    int nb_buckets_per_group_;

  public:
    CascadeHasher() {}

    // Creates the hashing projections (cascade of two level of hash codes)
    bool Init(std::mt19937& generator, const uint8_t nb_hash_code = 128, const uint8_t nb_bucket_groups = 6, const uint8_t nb_bits_per_bucket = 10)
    {
        nb_bucket_groups_ = nb_bucket_groups;
        nb_hash_code_ = nb_hash_code;
        nb_bits_per_bucket_ = nb_bits_per_bucket;
        nb_buckets_per_group_ = 1 << nb_bits_per_bucket;

        //
        // Box Muller transform is used in the original paper to get fast random number
        // from a normal distribution with <mean = 0> and <variance = 1>.
        // Here we use C++11 normal distribution random number generator
        std::normal_distribution<> d(0, 1);

        primary_hash_projection_.resize(nb_hash_code, nb_hash_code);

        // Initialize primary hash projection.
        for (int i = 0; i < nb_hash_code; ++i)
        {
            for (int j = 0; j < nb_hash_code; ++j)
                primary_hash_projection_(i, j) = d(generator);
        }

        // Initialize secondary hash projection.
        secondary_hash_projection_.resize(nb_bucket_groups);
        for (int i = 0; i < nb_bucket_groups; ++i)
        {
            secondary_hash_projection_[i].resize(nb_bits_per_bucket_, nb_hash_code);
            for (int j = 0; j < nb_bits_per_bucket_; ++j)
            {
                for (int k = 0; k < nb_hash_code; ++k)
                    secondary_hash_projection_[i](j, k) = d(generator);
            }
        }
        return true;
    }

    template<typename MatrixT>
    static Eigen::VectorXf GetZeroMeanDescriptor(const MatrixT& descriptions)
    {
        Eigen::VectorXf zero_mean_descriptor;
        if (descriptions.rows() == 0)
        {
            return zero_mean_descriptor;
        }
        // Compute the ZeroMean descriptor
        zero_mean_descriptor.setZero(descriptions.cols());
        const typename MatrixT::Index nbDescriptions = descriptions.rows();
        for (int i = 0; i < nbDescriptions; ++i)
        {
            for (int j = 0; j < descriptions.cols(); ++j)
                zero_mean_descriptor(j) += descriptions(i, j);
        }
        return zero_mean_descriptor / static_cast<double>(nbDescriptions);
    }

    template<typename MatrixT>
    HashedDescriptions CreateHashedDescriptions(const MatrixT& descriptions, const Eigen::VectorXf& zero_mean_descriptor) const
    {
        // Steps:
        //   1) Compute hash code and hash buckets (based on the zero_mean_descriptor).
        //   2) Construct buckets.

        HashedDescriptions hashed_descriptions;
        if (descriptions.rows() == 0)
        {
            return hashed_descriptions;
        }

        // Create hash codes for each description.
        {
            // Allocate space for hash codes.
            const typename MatrixT::Index nbDescriptions = descriptions.rows();
            hashed_descriptions.hashed_desc.resize(nbDescriptions);
            Eigen::VectorXf descriptor(descriptions.cols());
            for (int i = 0; i < nbDescriptions; ++i)
            {
                // Allocate space for each bucket id.
                hashed_descriptions.hashed_desc[i].bucket_ids.resize(nb_bucket_groups_);

                for (int k = 0; k < descriptions.cols(); ++k)
                {
                    descriptor(k) = descriptions(i, k);
                }
                descriptor -= zero_mean_descriptor;

                auto& hash_code = hashed_descriptions.hashed_desc[i].hash_code;
                hash_code = stl::dynamic_bitset(descriptions.cols());

                // Compute hash code.
                const Eigen::VectorXf primary_projection = primary_hash_projection_ * descriptor;
                for (int j = 0; j < nb_hash_code_; ++j)
                {
                    hash_code[j] = primary_projection(j) > 0;
                }

                // Determine the bucket index for each group.
                Eigen::VectorXf secondary_projection;
                for (int j = 0; j < nb_bucket_groups_; ++j)
                {
                    uint16_t bucket_id = 0;
                    secondary_projection = secondary_hash_projection_[j] * descriptor;

                    for (int k = 0; k < nb_bits_per_bucket_; ++k)
                    {
                        bucket_id = (bucket_id << 1) + (secondary_projection(k) > 0 ? 1 : 0);
                    }
                    hashed_descriptions.hashed_desc[i].bucket_ids[j] = bucket_id;
                }
            }
        }
        // Build the Buckets
        {
            hashed_descriptions.buckets.resize(nb_bucket_groups_);
            for (int i = 0; i < nb_bucket_groups_; ++i)
            {
                hashed_descriptions.buckets[i].resize(nb_buckets_per_group_);

                // Add the descriptor ID to the proper bucket group and id.
                for (int j = 0; j < hashed_descriptions.hashed_desc.size(); ++j)
                {
                    const uint16_t bucket_id = hashed_descriptions.hashed_desc[j].bucket_ids[i];
                    hashed_descriptions.buckets[i][bucket_id].push_back(j);
                }
            }
        }
        return hashed_descriptions;
    }

    // Matches two collection of hashed descriptions with a fast matching scheme
    // based on the hash codes previously generated.
    template<typename MatrixT, typename DistanceType>
    void Match_HashedDescriptions(const HashedDescriptions& hashed_descriptions1,
                                  const MatrixT& descriptions1,
                                  const HashedDescriptions& hashed_descriptions2,
                                  const MatrixT& descriptions2,
                                  IndMatches* pvec_indices,
                                  std::vector<DistanceType>* pvec_distances,
                                  const int NN = 2) const
    {
        typedef feature::L2_Vectorized<typename MatrixT::Scalar> MetricT;
        MetricT metric;

        static const int kNumTopCandidates = 10;

        // Preallocate the candidate descriptors container.
        std::vector<int> candidate_descriptors;
        candidate_descriptors.reserve(hashed_descriptions2.hashed_desc.size());

        // Preallocated hamming distances. Each column indicates the hamming distance
        // and the rows collect the descriptor ids with that
        // distance. num_descriptors_with_hamming_distance keeps track of how many
        // descriptors have that distance.
        Eigen::MatrixXi candidate_hamming_distances(hashed_descriptions2.hashed_desc.size(), nb_hash_code_ + 1);
        Eigen::VectorXi num_descriptors_with_hamming_distance(nb_hash_code_ + 1);

        // Preallocate the container for keeping euclidean distances.
        std::vector<std::pair<DistanceType, int>> candidate_euclidean_distances;
        candidate_euclidean_distances.reserve(kNumTopCandidates);

        // A preallocated vector to determine if we have already used a particular
        // feature for matching (i.e., prevents duplicates).
        std::vector<bool> used_descriptor(hashed_descriptions2.hashed_desc.size());

        typedef feature::Hamming<stl::dynamic_bitset::BlockType> HammingMetricType;
        static const HammingMetricType metricH = {};
        for (int i = 0; i < hashed_descriptions1.hashed_desc.size(); ++i)
        {
            candidate_descriptors.clear();
            num_descriptors_with_hamming_distance.setZero();
            candidate_euclidean_distances.clear();

            const auto& hashed_desc = hashed_descriptions1.hashed_desc[i];

            // Accumulate all descriptors in each bucket group that are in the same
            // bucket id as the query descriptor.
            for (int j = 0; j < nb_bucket_groups_; ++j)
            {
                const uint16_t bucket_id = hashed_desc.bucket_ids[j];
                for (const auto& feature_id : hashed_descriptions2.buckets[j][bucket_id])
                {
                    candidate_descriptors.emplace_back(feature_id);
                    used_descriptor[feature_id] = false;
                }
            }

            // Skip matching this descriptor if there are not at least NN candidates.
            if (candidate_descriptors.size() <= NN)
            {
                continue;
            }

            // Compute the hamming distance of all candidates based on the comp hash
            // code. Put the descriptors into buckets corresponding to their hamming
            // distance.
            for (const int candidate_id : candidate_descriptors)
            {
                if (!used_descriptor[candidate_id])  // avoid selecting the same candidate multiple times
                {
                    used_descriptor[candidate_id] = true;

                    const HammingMetricType::ResultType hamming_distance = metricH(hashed_desc.hash_code.data(),
                                                                                   hashed_descriptions2.hashed_desc[candidate_id].hash_code.data(),
                                                                                   hashed_desc.hash_code.num_blocks());
                    candidate_hamming_distances(num_descriptors_with_hamming_distance(hamming_distance)++, hamming_distance) = candidate_id;
                }
            }

            // Compute the euclidean distance of the k descriptors with the best hamming
            // distance.
            candidate_euclidean_distances.reserve(kNumTopCandidates);
            for (int j = 0; j < candidate_hamming_distances.cols() && (candidate_euclidean_distances.size() < kNumTopCandidates); ++j)
            {
                for (int k = 0; k < num_descriptors_with_hamming_distance(j) && (candidate_euclidean_distances.size() < kNumTopCandidates); ++k)
                {
                    const int candidate_id = candidate_hamming_distances(k, j);
                    const DistanceType distance = metric(descriptions2.row(candidate_id).data(), descriptions1.row(i).data(), descriptions1.cols());

                    candidate_euclidean_distances.emplace_back(distance, candidate_id);
                }
            }

            // Assert that each query is having at least NN retrieved neighbors
            if (candidate_euclidean_distances.size() >= NN)
            {
                // Find the top NN candidates based on euclidean distance.
                std::partial_sort(
                  candidate_euclidean_distances.begin(), candidate_euclidean_distances.begin() + NN, candidate_euclidean_distances.end());
                // save resulting neighbors
                for (int l = 0; l < NN; ++l)
                {
                    pvec_distances->emplace_back(candidate_euclidean_distances[l].first);
                    pvec_indices->emplace_back(IndMatch(i, candidate_euclidean_distances[l].second));
                }
            }
            // else -> too few candidates... (save no one)
        }
    }

  private:
    // Primary hashing function.
    Eigen::MatrixXf primary_hash_projection_;

    // Secondary hashing function.
    std::vector<Eigen::MatrixXf> secondary_hash_projection_;
};

}  // namespace matching
}  // namespace aliceVision
