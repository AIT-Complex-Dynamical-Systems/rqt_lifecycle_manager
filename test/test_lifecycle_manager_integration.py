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

"""Integration tests against real lifecycle nodes running in separate processes."""

import os
import signal
import subprocess
import sys
import threading
import time

import pytest
from python_qt_binding.QtCore import QTimer
from rclpy.executors import MultiThreadedExecutor

from rqt_lifecycle_manager.lifecycle_manager import UNREACHABLE

LIFECYCLE_NODE = os.path.join(os.path.dirname(__file__), "lifecycle_node.py")
DISCOVERY_TIMEOUT = 15.0  # seconds


@pytest.fixture
def start_lifecycle_node():
    processes = []

    def start(name):
        process = subprocess.Popen(
            [sys.executable, LIFECYCLE_NODE, name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        processes.append(process)
        return process

    yield start

    for process in processes:
        process.send_signal(signal.SIGCONT)
        process.kill()
        process.wait()


@pytest.fixture
def executor(make_plugin):
    # Spin the node in a background thread, like rqt_gui_py does
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=executor.spin, daemon=True)
    yield executor, thread
    executor.shutdown()
    thread.join(timeout=5.0)


@pytest.fixture
def plugin(make_plugin, executor):
    plugin = make_plugin(update_freq=5.0, service_timeout=0.5)
    executor[0].add_node(plugin._node)
    executor[1].start()
    return plugin


@pytest.fixture
def wait_for(qapp):
    def wait(predicate, timeout=DISCOVERY_TIMEOUT):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            qapp.processEvents()
            time.sleep(0.01)
        return predicate()

    return wait


@pytest.fixture
def gui_stall_monitor():
    """Measure the longest gap between Qt timer events of a 50 ms timer."""
    monitor = {"last": time.monotonic(), "max_gap": 0.0}

    def beat():
        now = time.monotonic()
        monitor["max_gap"] = max(monitor["max_gap"], now - monitor["last"])
        monitor["last"] = now

    timer = QTimer()
    timer.timeout.connect(beat)
    timer.start(50)
    yield monitor
    timer.stop()


def state(plugin, node_name):
    return {n.name: n.state for n in plugin._lc_nodes}.get(node_name)


def test_shows_discovered_node(plugin, executor, start_lifecycle_node, wait_for):
    start_lifecycle_node("lc_discovered")

    assert wait_for(lambda: state(plugin, "/lc_discovered") == "unconfigured")
    # the node must stay with the rqt executor, other plugins depend on it
    assert plugin._node.executor is executor[0]


def test_configure_and_activate(plugin, start_lifecycle_node, wait_for):
    start_lifecycle_node("lc_activated")
    assert wait_for(lambda: state(plugin, "/lc_activated") == "unconfigured")

    plugin._call_transitions("/lc_activated", ["configure", "activate"])

    assert wait_for(lambda: state(plugin, "/lc_activated") == "active", timeout=5.0)


def test_frozen_node_does_not_block_gui(
    plugin, start_lifecycle_node, wait_for, gui_stall_monitor
):
    process = start_lifecycle_node("lc_frozen")
    start_lifecycle_node("lc_healthy")
    assert wait_for(lambda: state(plugin, "/lc_frozen") == "unconfigured")
    assert wait_for(lambda: state(plugin, "/lc_healthy") == "unconfigured")

    process.send_signal(signal.SIGSTOP)
    assert wait_for(lambda: state(plugin, "/lc_frozen") == UNREACHABLE, timeout=5.0)
    plugin._call_transitions("/lc_frozen", ["configure"])
    plugin._call_transitions("/lc_healthy", ["configure"])
    assert wait_for(lambda: state(plugin, "/lc_healthy") == "inactive", timeout=5.0)
    assert gui_stall_monitor["max_gap"] < 0.3

    process.send_signal(signal.SIGCONT)
    assert wait_for(lambda: state(plugin, "/lc_frozen") != UNREACHABLE, timeout=5.0)
