# Copyright 2024 AIT - Austrian Institute of Technology GmbH
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
import sys
import time
from collections import namedtuple

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractTableModel, Qt, QTimer, Signal
from python_qt_binding.QtGui import QBrush, QFont, QIcon
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QWidget
from qt_gui.plugin import Plugin
from ros2lifecycle.api import get_node_names

# Define a simple structure with fields 'name' and 'state'
NodeState = namedtuple("NodeState", ["name", "state"])

# Shown instead of the lifecycle state if the node does not respond
UNREACHABLE = "unreachable"


class LifecycleManager(Plugin):
    """Graphical frontend for interacting with lifecycle nodes."""

    # Emitted from the executor thread when a get_state request is done
    _get_state_done = Signal(str, object)  # node name, future
    # Emitted from the executor thread when a change_state request is done
    _change_state_done = Signal(str, object, object)  # node name, future, next labels

    _default_update_freq = 1.0  # Hz
    _default_service_timeout = 1.0  # seconds
    _params_prefix = "rqt_lifecycle_manager"

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("LifecycleManager")

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_lifecycle_manager"),
            "resource",
            "lifecycle_manager.ui",
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("LifecycleManagerUi")

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"{self._widget.windowTitle()} {context.serial_number()}"
            )
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize members
        self._lc_node_names = []  # list of lifecycle node names
        self._lc_nodes = []  # list of lifecycle node status
        self._table_model = None
        self._get_state_clients = {}  # node name -> GetState client
        self._change_state_clients = {}  # node name -> ChangeState client
        self._get_state_requests = {}  # node name -> (future, send time)
        self._states = {}  # node name -> state label
        self._get_state_done.connect(self._on_get_state_done, Qt.QueuedConnection)
        self._change_state_done.connect(self._on_change_state_done, Qt.QueuedConnection)

        # Store reference to node
        self._node = context.node

        # Load plugin behavior from ROS parameters.
        self._update_freq = self._get_positive_float_param(
            "update_freq", self._default_update_freq
        )
        self._service_timeout = self._get_positive_float_param(
            "service_timeout", self._default_service_timeout
        )

        # lc node state icons
        path = get_package_share_directory("rqt_lifecycle_manager")
        self._icons = {
            "active": QIcon(f"{path}/resource/led_green.png"),
            "finalized": QIcon(f"{path}/resource/led_off.png"),
            "inactive": QIcon(f"{path}/resource/led_red.png"),
            "unconfigured": QIcon(f"{path}/resource/led_off.png"),
        }

        # lc nodes display
        table_view = self._widget.table_view
        table_view.setContextMenuPolicy(Qt.CustomContextMenu)
        table_view.customContextMenuRequested.connect(self._on_lc_node_menu)

        header = table_view.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setContextMenuPolicy(Qt.CustomContextMenu)

        # Timer for listing nodes and updating their states
        self._update_timer = QTimer(self)
        self._update_timer.setInterval(int(1000.0 / self._update_freq))
        self._update_timer.timeout.connect(self._update)
        self._update_timer.start()

    def _get_positive_float_param(self, param_name, default_value):
        """Return a positive float from ROS parameters, falling back to default."""
        full_name = f"{self._params_prefix}.{param_name}"

        if not self._node.has_parameter(full_name):
            self._node.declare_parameter(full_name, default_value)

        value = self._node.get_parameter(full_name).value
        try:
            value = float(value)
        except (TypeError, ValueError):
            print(
                (
                    f"Invalid parameter '{full_name}'={value!r}; "
                    f"using default {default_value}."
                ),
                file=sys.stderr,
            )
            return default_value

        if value <= 0.0:
            print(
                (
                    f"Parameter '{full_name}' must be > 0, got {value}; "
                    f"using default {default_value}."
                ),
                file=sys.stderr,
            )
            return default_value

        return value

    def shutdown_plugin(self):
        self._update_timer.stop()
        for future, _ in self._get_state_requests.values():
            future.cancel()
        self._get_state_requests.clear()
        for clients in (self._get_state_clients, self._change_state_clients):
            for client in clients.values():
                self._node.destroy_client(client)
            clients.clear()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def _update(self):
        self._update_node_list()
        self._update_nodes_state()

    def _update_node_list(self):
        node_names = self._list_lc_nodes()

        if self._lc_node_names != node_names:
            self._lc_node_names = node_names
            self._destroy_stale_clients({n.full_name for n in node_names})

    def _update_nodes_state(self):
        node_names = [lc_node.full_name for lc_node in self._lc_node_names]
        for node_name in self._states.keys() - set(node_names):
            del self._states[node_name]
        for node_name in self._get_state_requests.keys() - set(node_names):
            self._get_state_requests.pop(node_name)[0].cancel()

        now = time.monotonic()
        for node_name in node_names:
            request = self._get_state_requests.get(node_name)
            if request is not None:
                future, stamp = request
                if now - stamp < self._service_timeout:
                    continue  # still waiting for the previous response
                del self._get_state_requests[node_name]
                future.cancel()
                self._set_state_error(
                    node_name,
                    TimeoutError("Timed out while waiting for get_state response"),
                )

            client = self._get_client(
                self._get_state_clients, GetState, node_name, "get_state"
            )
            if not client.service_is_ready():
                self._set_state_error(
                    node_name,
                    TimeoutError(
                        "get_state service is not available "
                        "(node might have disappeared)"
                    ),
                )
                continue

            # The response is handled in the GUI thread, the rqt executor spins the node
            future = client.call_async(GetState.Request())
            self._get_state_requests[node_name] = (future, now)
            future.add_done_callback(
                lambda f, n=node_name: self._get_state_done.emit(n, f)
            )

        self._show_lc_nodes()

    def _on_get_state_done(self, node_name, future):
        request = self._get_state_requests.get(node_name)
        if request is None or request[0] is not future:
            return  # response of a cancelled or outdated request
        del self._get_state_requests[node_name]

        error = future.exception()
        if error is None and future.result() is None:
            error = RuntimeError("get_state service call failed")
        if error is not None:
            self._set_state_error(node_name, error)
        else:
            self._states[node_name] = future.result().current_state.label
        self._show_lc_nodes()

    def _set_state_error(self, node_name, error):
        # Only report the first error to avoid spamming
        if self._states.get(node_name) != UNREACHABLE:
            self._states[node_name] = UNREACHABLE
            print(
                f"Exception while calling service of node '{node_name}': {error}",
                file=sys.stderr,
            )

    def _destroy_stale_clients(self, node_names):
        """Destroy the service clients of nodes that left the graph."""
        for clients in (self._get_state_clients, self._change_state_clients):
            for node_name in list(clients.keys() - node_names):
                self._node.destroy_client(clients.pop(node_name))

    def _get_client(self, clients, srv_type, node_name, service):
        """Return a cached service client, creating it on first use."""
        if node_name not in clients:
            clients[node_name] = self._node.create_client(
                srv_type, f"{node_name}/{service}"
            )
        return clients[node_name]

    def _list_lc_nodes(self):
        """
        List the lifecycle nodes.

        @rtype [str]
        """
        try:
            node_names = get_node_names(node=self._node, include_hidden_nodes=False)
            return node_names
        except RuntimeError as e:
            print(e)
            return []

    def _show_lc_nodes(self):
        self._lc_nodes = [
            NodeState(name=node_name, state=self._states[node_name])
            for node_name in sorted(self._states.keys())
        ]
        table_view = self._widget.table_view
        self._table_model = LifecycleNodeTable(self._lc_nodes, self._icons)
        table_view.setModel(self._table_model)

    def _on_lc_node_menu(self, pos):
        # Get data of selected node
        row = self._widget.table_view.rowAt(pos.y())
        if row < 0:
            return  # Cursor is not under a valid item

        lc_node = self._lc_nodes[row]

        # Show context menu
        menu = QMenu(self._widget.table_view)
        if lc_node.state == "active":
            action_deactivate = menu.addAction(self._icons["inactive"], "Deactivate")
            action_unspawn = menu.addAction(
                self._icons["unconfigured"], "Deactivate and cleanup"
            )
            action_shutdown = menu.addAction(self._icons["finalized"], "Shutdown")
        elif lc_node.state == "inactive":
            action_activate = menu.addAction(self._icons["active"], "Activate")
            action_cleanup = menu.addAction(self._icons["unconfigured"], "Cleanup")
            action_shutdown = menu.addAction(self._icons["finalized"], "Shutdown")
        elif lc_node.state == "unconfigured":
            action_configure = menu.addAction(self._icons["inactive"], "Configure")
            action_spawn = menu.addAction(
                self._icons["active"], "Configure and Activate"
            )
            action_shutdown = menu.addAction(self._icons["finalized"], "Shutdown")
        else:
            pass  # finalized

        action = menu.exec_(self._widget.table_view.mapToGlobal(pos))

        # Evaluate user action
        if lc_node.state == "active":
            if action is action_deactivate:
                self._call_transitions(lc_node.name, ["deactivate"])
            elif action is action_shutdown:
                self._call_transitions(lc_node.name, ["shutdown"])
            elif action is action_unspawn:
                self._call_transitions(lc_node.name, ["deactivate", "cleanup"])
        elif lc_node.state == "inactive":
            if action is action_activate:
                self._call_transitions(lc_node.name, ["activate"])
            elif action is action_cleanup:
                self._call_transitions(lc_node.name, ["cleanup"])
            elif action is action_shutdown:
                self._call_transitions(lc_node.name, ["shutdown"])
        elif lc_node.state == "unconfigured":
            if action is action_configure:
                self._call_transitions(lc_node.name, ["configure"])
            elif action is action_shutdown:
                self._call_transitions(lc_node.name, ["shutdown"])
            elif action is action_spawn:
                self._call_transitions(lc_node.name, ["configure", "activate"])
        else:
            pass  # finalized

    def _call_transitions(self, node_name, transition_labels):
        """Request the transitions in order, stopping at the first failure."""
        client = self._get_client(
            self._change_state_clients, ChangeState, node_name, "change_state"
        )
        if not client.service_is_ready():
            self._print_transition_error(
                node_name,
                TimeoutError(
                    "change_state service is not available "
                    "(node might have disappeared)"
                ),
            )
            return

        request = ChangeState.Request()
        request.transition = Transition(label=transition_labels[0])
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._change_state_done.emit(node_name, f, transition_labels[1:])
        )
        QTimer.singleShot(
            int(1000.0 * self._service_timeout),
            lambda: self._on_change_state_timeout(node_name, future),
        )

    def _on_change_state_timeout(self, node_name, future):
        if not future.done() and not future.cancelled():
            future.cancel()
            self._print_transition_error(
                node_name,
                TimeoutError("Timed out while waiting for change_state response"),
            )

    def _on_change_state_done(self, node_name, future, transition_labels):
        if future.cancelled():
            return  # timed out

        error = future.exception()
        if error is None and future.result() is None:
            error = RuntimeError("change_state service call failed")
        if error is not None:
            self._print_transition_error(node_name, error)
        elif not future.result().success:
            print("Transitioning failed", file=sys.stderr)
        else:
            print("Transitioning successful")
            if transition_labels:
                self._call_transitions(node_name, transition_labels)

    def _print_transition_error(self, node_name, error):
        print(
            f"Exception while calling service of node '{node_name}': {error}",
            file=sys.stderr,
        )


class LifecycleNodeTable(QAbstractTableModel):
    """
    Model containing lifecycle node information for tabular display.

    The model allows display of basic read-only information
    """

    def __init__(self, node_info, icons, parent=None):
        QAbstractTableModel.__init__(self, parent)
        self._data = node_info
        self._icons = icons

    def rowCount(self, parent):
        return len(self._data)

    def columnCount(self, parent):
        return 2

    def headerData(self, col, orientation, role):
        if orientation != Qt.Horizontal or role != Qt.DisplayRole:
            return None
        if col == 0:
            return "node"
        elif col == 1:
            return "state"

    def data(self, index, role):
        if not index.isValid():
            return None

        lc_node = self._data[index.row()]

        if role == Qt.DisplayRole:
            if index.column() == 0:
                return lc_node.name
            elif index.column() == 1:
                return lc_node.state or "not loaded"

        if role == Qt.DecorationRole and index.column() == 0:
            return self._icons.get(lc_node.state)

        if role == Qt.FontRole and index.column() == 0:
            bf = QFont()
            bf.setBold(True)
            return bf

        if role == Qt.ForegroundRole and lc_node.state == UNREACHABLE:
            return QBrush(Qt.gray)

        if role == Qt.TextAlignmentRole and index.column() == 1:
            return Qt.AlignCenter
