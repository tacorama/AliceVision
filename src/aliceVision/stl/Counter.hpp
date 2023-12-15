// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <unordered_map>

namespace stl {

template <class Key>
class Counter : public std::map<Key, size_t>
{
public:
    size_t & operator[](const Key & key)
    {
        auto it = this->find(key);

        if (it == this->end())
        {
            this->emplace(key, 0);
            it = this->find(key);
        }

        return it->second;
    }
};

}
