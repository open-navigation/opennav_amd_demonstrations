#! /usr/bin/env python3
# Copyright 2024 Open Naviation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import random


"""
Pseudo-cloud dispatcher interface to get next pick and drop location tasks
"""


class Dispatcher():

    def __init__(self, picking_locations, goods_bins):
        self.picking_locations = picking_locations
        self.goods_bins = goods_bins

    def get_next_picks(self, num_picks=1):
        picks = []
        for _ in range(num_picks):
            picks.append(self.get_next_pick())
        return picks

    def get_next_drops(self, num_drops=1):
        drops = []
        for _ in range(num_drops):
            drops.append(self.get_next_drop())
        return drops

    # Would be replaced by request from centralized system
    def get_next_pick(self):
        pick_shelf = random.choice(list(self.picking_locations.keys()))
        pick_slot = random.choice(list(self.picking_locations[pick_shelf].keys()))
        return self.picking_locations[pick_shelf][pick_slot]

    # Would be replaced by request from centralized system
    def get_next_drop(self):
        return self.goods_bins[random.choice(list(self.goods_bins.keys()))]
