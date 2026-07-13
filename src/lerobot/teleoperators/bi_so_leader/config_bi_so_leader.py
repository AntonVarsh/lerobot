#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass

from ..config import TeleoperatorConfig
from ..so_leader import SOLeaderConfig


@TeleoperatorConfig.register_subclass("bi_so_leader")
@dataclass
class BiSOLeaderConfig(TeleoperatorConfig):
    """Configuration class for Bi SO Leader teleoperators."""

    left_arm_config: SOLeaderConfig
    right_arm_config: SOLeaderConfig


# Distinct class (not an alias) so draccus' `.type` resolution picks the right one -- see the
# same note in `so_leader/config_so_leader.py`. `left_arm_config`/`right_arm_config` stay typed
# as the shared `SOLeaderConfig` since the per-arm motor layout is decided by `BiSO107Leader`
# (which arm class it instantiates), not by this config.
@TeleoperatorConfig.register_subclass("bi_so107_leader")
@dataclass
class BiSO107LeaderConfig(TeleoperatorConfig):
    """Configuration class for Bi SO-107 Leader teleoperators."""

    left_arm_config: SOLeaderConfig
    right_arm_config: SOLeaderConfig
