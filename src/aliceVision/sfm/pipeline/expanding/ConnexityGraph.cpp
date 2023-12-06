// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ConnexityGraph.hpp"

namespace aliceVision {
namespace sfm {

bool ConnexityGraph::build(const sfmData::SfMData & sfmData, const std::set<IndexT> & viewsOfInterest)
{
    std::map<std::pair<IndexT, IndexT>, unsigned int> covisibility;

    std::vector<IndexT> views;
    for (const auto & pv : sfmData.getViews())
    {
        views.push_back(pv.first);
    }
    

    for (int i = 0; i < views.size(); i++)
    {
        IndexT viewI = views[i];
        
        for (int j = i + 1; j < views.size(); j++)
        {
            IndexT viewJ = views[j];

            IndexT min = std::min(viewI, viewJ);
            IndexT max = std::max(viewI, viewJ);

            covisibility[std::make_pair(min, max)] = 0; 
        }
    }

    for (const auto & pl : sfmData.getLandmarks())
    {
        std::vector<IndexT> observedViews;
        for (const auto & po : pl.second.observations)
        {
            observedViews.push_back(po.first);
        }

        for (int i = 0; i < observedViews.size(); i++)
        {
            IndexT viewI = observedViews[i];

            for (int j =  i + 1; j < observedViews.size(); j++)
            {
                IndexT viewJ = observedViews[j];
                
                IndexT min = std::min(viewI, viewJ);
                IndexT max = std::max(viewI, viewJ);

                covisibility[std::make_pair(min, max)]++; 
            }
        }
    }

    for (const auto & item : covisibility)
    {
        if (item.second < 50)
        {
            continue;
        }

        
    }

    return true;
}

}
}