# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from requests import options
import carb
import argparse
import omni.usd
import asyncio
import omni.client
import omni.kit.async_engine
import omni.timeline

import sys
import os

script_dir = os.path.join(os.path.dirname(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

from build_stage import create_scene


def main():
    parser = argparse.ArgumentParser()

    # Add the --start-on-play option
    # If --start-on-play is specified, it sets the value to True
    parser.add_argument('--start-on-play', action='store_true',
                        help='If present, to true.')

    try:
        options = parser.parse_args()
    except Exception as e:
        carb.log_error(str(e))
        return

    omni.kit.async_engine.run_coroutine(build_stage_async(options))


async def build_stage_async(options):
    timeline_interface = None
    if options.start_on_play:
        timeline_interface = omni.timeline.get_timeline_interface()

    # Load your USDs
    env_usd_path = os.path.join(script_dir, "..", "assets", "KIBOU_ISS.usd")
    robot_usd_path = os.path.join(script_dir, "..", "assets", "Intball2", "INTBALL2.usd")

    # Create the scene and get World object
    await create_scene(env_usd_path, robot_usd_path)

    if timeline_interface is not None:
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        timeline_interface.play()
        print("Stage loaded and simulation is playing.")
    pass

    broken_url = omni.client.break_url(env_usd_path)
    if broken_url.scheme == 'omniverse':
        # Attempt to connect to nucleus server before opening stage
        try:
            from omni.kit.widget.nucleus_connector import get_nucleus_connector
            nucleus_connector = get_nucleus_connector()
        except Exception:
            carb.log_warn("Open stage: Could not import Nucleus connector.")
            return

        server_url = omni.client.make_url(scheme='omniverse', host=broken_url.host)
        nucleus_connector = get_nucleus_connector()
        nucleus_connector.connect(
            broken_url.host, server_url,
            on_success_fn=lambda *_: print(f"Connected to Nucleus server: {server_url}"),
            on_failed_fn=lambda *_: carb.log_error(f"Failed to connect to Nucleus: {server_url}")
        )


main()
