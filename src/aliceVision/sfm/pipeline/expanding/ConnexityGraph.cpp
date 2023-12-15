// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ConnexityGraph.hpp"
#include <aliceVision/stl/Counter.hpp>

namespace aliceVision {
namespace sfm {

bool ConnexityGraph::build(const sfmData::SfMData & sfmData, const std::set<IndexT> & viewsOfInterest)
{
    //Create a list of reconstructed views
    std::vector<IndexT> views;
    for (const auto & pv : sfmData.getViews())
    {
        if (sfmData.isPoseAndIntrinsicDefined(pv.first))
        {
            views.push_back(pv.first);


            lemon::ListGraph::Node newNode = _graph.addNode();

            //Store which node relate to which view
            _nodePerViewId[pv.first] = newNode;
            _viewIdPerNode[newNode] = pv.first;
        }
    }


    //Build a list of landmarks index per view
    std::map<IndexT, std::set<IndexT>> landmarksPerView;
    for (const auto & pl : sfmData.getLandmarks())
    {
        for (const auto & po : pl.second.observations)
        {
            landmarksPerView[po.first].insert(pl.first);
        }
    }
    
    //For all possible unique pairs
    std::map<IndexT, std::vector<std::pair<IndexT, size_t>>> covisibility;
    for (int idref = 0; idref < views.size(); idref++)
    {
        IndexT viewRef = views[idref];

        const auto & ptref = landmarksPerView[viewRef];
        
        for (int idcur = idref + 1; idcur < views.size(); idcur++)
        {
            IndexT viewCur = views[idcur];

            const auto & ptcur = landmarksPerView[viewCur];

            std::vector<IndexT> intersection;
            std::set_intersection(ptref.begin(), ptref.end(), ptcur.begin(), ptcur.end(), 
                                    std::back_inserter(intersection));

            size_t s = intersection.size();
            if (s == 0)
            {
                continue;
            }

            covisibility[viewRef].push_back(std::make_pair(viewCur, s));
            covisibility[viewCur].push_back(std::make_pair(viewRef, s));
        }
    }

    for (const auto & item : covisibility)
    {

    }

    /*
    for (const auto & item : covisibility)
    {
        if (item.second == 0)
        {
            continue;
        }

        IndexT viewId1 = item.first.first;
        IndexT viewId2 = item.first.second;

        const lemon::ListGraph::Node & node1 = _nodePerViewId[viewId1];
        const lemon::ListGraph::Node & node2 = _nodePerViewId[viewId2];

        _graph.addEdge(node1, node1);
    }
    */

    return true;
}

}
}