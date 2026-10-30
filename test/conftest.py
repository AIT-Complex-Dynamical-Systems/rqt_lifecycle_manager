# Copyright 2026 AIT - Austrian Institute of Technology GmbH
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

import os
import random
import time

# Run without display and isolated from other ROS nodes on the network.
# Must be set before rclpy is initialized, subprocesses inherit it.
os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["ROS_DOMAIN_ID"] = str(random.randint(150, 230))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import pytest  # noqa: E402
import rclpy  # noqa: E402
from python_qt_binding.QtCore import QObject  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from rqt_lifecycle_manager.lifecycle_manager import LifecycleManager  # noqa: E402


class FakeContext(QObject):
    """Minimal stand-in for the qt_gui plugin context."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.widgets = []

    def serial_number(self):
        return 1

    def add_widget(self, widget):
        self.widgets.append(widget)


@pytest.fixture(scope="session")
def qapp():
    yield QApplication.instance() or QApplication([])


@pytest.fixture(scope="session")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def process_events(qapp):
    """Process Qt events for the given duration, at least once."""

    def process(duration=0.0):
        deadline = time.monotonic() + duration
        qapp.processEvents()
        while time.monotonic() < deadline:
            qapp.processEvents()
            time.sleep(0.005)

    return process


@pytest.fixture
def make_plugin(qapp, ros):
    """Create plugins on a fresh node, with optional plugin parameters."""
    created = []

    def make(**params):
        node = rclpy.create_node(
            "rqt_lifecycle_manager_test",
            parameter_overrides=[
                Parameter(f"rqt_lifecycle_manager.{name}", value=value)
                for name, value in params.items()
            ],
        )
        # keep the context alive, it is the Qt parent of the plugin
        context = FakeContext(node)
        plugin = LifecycleManager(context)
        created.append((context, plugin, node))
        return plugin

    yield make

    for _, plugin, node in created:
        plugin.shutdown_plugin()
        node.destroy_node()
