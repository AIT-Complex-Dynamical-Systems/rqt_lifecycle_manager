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

"""Unit tests with a fake ROS graph and fake service clients."""

import types

import pytest
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.task import Future

from rqt_lifecycle_manager import lifecycle_manager
from rqt_lifecycle_manager.lifecycle_manager import (
    UNREACHABLE,
    LifecycleNodeTable,
    NodeState,
)


class FakeClient:
    def __init__(self, srv_name, ready=True):
        self.srv_name = srv_name
        self.ready = ready
        self.destroyed = False
        self.requests = []  # (request, future)

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.requests.append((request, future))
        return future


class FakeGraph:
    """Replaces the lifecycle node discovery and service clients of the plugin."""

    def __init__(self, plugin, monkeypatch):
        self.node_names = []
        self.unavailable = set()  # node names whose new clients are not ready
        self.clients = {}  # service name -> FakeClient
        monkeypatch.setattr(
            plugin,
            "_list_lc_nodes",
            lambda: [types.SimpleNamespace(full_name=n) for n in self.node_names],
        )
        monkeypatch.setattr(plugin._node, "create_client", self._create_client)
        monkeypatch.setattr(plugin._node, "destroy_client", self._destroy_client)

    def _create_client(self, srv_type, srv_name):
        node_name = srv_name.rsplit("/", 1)[0]
        self.clients[srv_name] = FakeClient(srv_name, node_name not in self.unavailable)
        return self.clients[srv_name]

    def _destroy_client(self, client):
        client.destroyed = True

    def client(self, node_name, service="get_state"):
        return self.clients[f"{node_name}/{service}"]


@pytest.fixture
def clock(monkeypatch):
    clock = types.SimpleNamespace(now=0.0)
    monkeypatch.setattr(
        lifecycle_manager, "time", types.SimpleNamespace(monotonic=lambda: clock.now)
    )
    return clock


@pytest.fixture
def plugin(make_plugin):
    plugin = make_plugin(service_timeout=1.0)
    plugin._update_timer.stop()  # updates are triggered manually
    return plugin


@pytest.fixture
def graph(plugin, monkeypatch):
    return FakeGraph(plugin, monkeypatch)


def respond_state(client, label, index=-1):
    response = GetState.Response()
    response.current_state.label = label
    client.requests[index][1].set_result(response)


def respond_transition(client, success, index=-1):
    client.requests[index][1].set_result(ChangeState.Response(success=success))


def states(plugin):
    return {lc_node.name: lc_node.state for lc_node in plugin._lc_nodes}


def test_shows_states(plugin, graph, clock, process_events):
    graph.node_names = ["/a", "/b"]
    plugin._update()
    respond_state(graph.client("/a"), "active")
    respond_state(graph.client("/b"), "unconfigured")
    process_events()

    assert plugin._lc_nodes == [
        NodeState(name="/a", state="active"),
        NodeState(name="/b", state="unconfigured"),
    ]


def test_one_pending_request_per_node(plugin, graph, clock, process_events):
    graph.node_names = ["/a"]
    plugin._update()
    clock.now += 0.5
    plugin._update()

    assert len(graph.client("/a").requests) == 1

    respond_state(graph.client("/a"), "active")
    process_events()
    plugin._update()

    assert len(graph.client("/a").requests) == 2


def test_clients_are_reused(plugin, graph, clock, process_events):
    graph.node_names = ["/a"]
    for _ in range(3):
        plugin._update()
        respond_state(graph.client("/a"), "active")
        process_events()

    assert list(graph.clients) == ["/a/get_state"]
    assert len(graph.client("/a").requests) == 3


def test_timeout_marks_unreachable_and_retries(plugin, graph, clock, process_events):
    graph.node_names = ["/a"]
    plugin._update()
    respond_state(graph.client("/a"), "active")
    process_events()
    plugin._update()
    clock.now += 1.5
    plugin._update()
    process_events()

    requests = graph.client("/a").requests
    assert requests[1][1].cancelled()
    assert len(requests) == 3
    assert states(plugin) == {"/a": UNREACHABLE}


def test_outdated_response_is_ignored(plugin, graph, clock, process_events):
    graph.node_names = ["/a"]
    plugin._update()
    clock.now += 1.5
    plugin._update()  # times out the first request and sends a new one
    outdated = Future()
    outdated.set_result(GetState.Response())

    plugin._on_get_state_done("/a", outdated)

    assert states(plugin) == {"/a": UNREACHABLE}
    assert plugin._get_state_requests["/a"][0] is graph.client("/a").requests[1][1]


def test_unavailable_service_is_reported_once(
    plugin, graph, clock, process_events, capsys
):
    graph.node_names = ["/a"]
    graph.unavailable = {"/a"}
    for _ in range(3):
        plugin._update()

    assert states(plugin) == {"/a": UNREACHABLE}
    assert capsys.readouterr().err.count("not available") == 1


def test_recovers_from_unreachable(plugin, graph, clock, process_events):
    graph.node_names = ["/a"]
    plugin._update()
    graph.client("/a").ready = False
    clock.now += 1.5
    plugin._update()
    assert states(plugin) == {"/a": UNREACHABLE}

    graph.client("/a").ready = True
    plugin._update()
    respond_state(graph.client("/a"), "inactive")
    process_events()

    assert states(plugin) == {"/a": "inactive"}


def test_node_leaving_graph_is_cleaned_up(plugin, graph, clock, process_events):
    graph.node_names = ["/a", "/b"]
    plugin._update()
    respond_state(graph.client("/a"), "active")
    process_events()
    plugin._call_transitions("/b", ["configure"])

    graph.node_names = ["/a"]
    plugin._update()
    process_events()

    assert states(plugin) == {"/a": "active"}
    assert graph.client("/b").destroyed
    assert graph.client("/b", "change_state").destroyed
    assert graph.client("/b").requests[0][1].cancelled()
    assert not graph.client("/a").destroyed


def test_transitions_are_chained(plugin, graph, process_events):
    plugin._call_transitions("/a", ["configure", "activate"])
    client = graph.client("/a", "change_state")
    assert [r.transition.label for r, _ in client.requests] == ["configure"]

    respond_transition(client, success=True)
    process_events()
    assert [r.transition.label for r, _ in client.requests] == [
        "configure",
        "activate",
    ]


def test_transitions_stop_on_failure(plugin, graph, process_events, capsys):
    plugin._call_transitions("/a", ["configure", "activate"])
    client = graph.client("/a", "change_state")

    respond_transition(client, success=False)
    process_events()

    assert len(client.requests) == 1
    assert "Transitioning failed" in capsys.readouterr().err


def test_transition_to_unavailable_service(plugin, graph, capsys):
    graph.unavailable = {"/a"}
    plugin._call_transitions("/a", ["configure"])

    assert graph.client("/a", "change_state").requests == []
    assert "not available" in capsys.readouterr().err


def test_transition_timeout(plugin, graph, process_events, capsys):
    plugin._service_timeout = 0.05
    plugin._call_transitions("/a", ["configure", "activate"])
    process_events(0.2)

    client = graph.client("/a", "change_state")
    assert client.requests[0][1].cancelled()
    assert len(client.requests) == 1
    assert "Timed out while waiting for change_state" in capsys.readouterr().err


def test_table_model_keeps_rows_when_only_states_change(qapp):
    model = LifecycleNodeTable([NodeState("/a", "active")], icons={})
    resets, changes = [], []
    model.modelReset.connect(lambda: resets.append(True))
    model.dataChanged.connect(lambda *args: changes.append(True))

    model.set_node_info([NodeState("/a", "active")])
    assert (resets, changes) == ([], [])

    model.set_node_info([NodeState("/a", "inactive")])
    assert (resets, changes) == ([], [True])

    model.set_node_info([NodeState("/a", "inactive"), NodeState("/b", "active")])
    assert (resets, changes) == ([True], [True])
    assert model.rowCount(None) == 2
