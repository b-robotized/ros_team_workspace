#!/bin/bash

# Copyright (c) 2021-2026, b»robotized group
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Lists currently running ROS 2 nodes and kills their processes.
#
# Some nodes never have their ROS node name anywhere in their own process
# command line - e.g. any node hosted inside a Gazebo/Ignition simulation via
# a ros2_control plugin (gz_ros_control), a plain `ros2_control_node`
# (controller_manager's own executable name), or a component container. A
# plain name-based `ps` grep can't find those, so they get left running.
# When a node can't be resolved by name, this also checks for known
# multi-node host processes as a fallback.

echo "🔍 Listing ROS 2 nodes..."
nodes=$(ros2 node list)

if [ -z "$nodes" ]; then
  echo "✅ No ROS 2 nodes are currently running."
  exit 0
fi

echo "🧠 Resolving PIDs of ROS 2 nodes..."
unresolved_nodes=()
for node in $nodes; do
  echo "➡ Node: $node"
  # Match the node name at word boundaries, not as an unanchored substring -
  # avoids false positives like a --params-file path that happens to contain
  # the node name (e.g. controller_manager_and_generic_controllers.yaml).
  pids=$(ps -eo pid,cmd | grep -E "\b${node#/}\b" | grep -v grep | awk '{print $1}')
  if [ -z "$pids" ]; then
    echo "  ⚠ No PID found for $node"
    unresolved_nodes+=("$node")
  else
    for pid in $pids; do
      echo "  ❌ Killing PID $pid"
      kill -9 "$pid"
    done
  fi
done

if [ "${#unresolved_nodes[@]}" -gt 0 ]; then
  echo ""
  echo "🕵️ ${#unresolved_nodes[@]} node(s) not matched by name (${unresolved_nodes[*]})."
  echo "   Checking known multi-node host processes (Gazebo/Ignition simulation,"
  echo "   ros2_control_node, component containers)..."

  host_pids=$(ps -eo pid,cmd | grep -E "gz sim|gzserver|gzclient|ign gazebo|ros2_control_node|component_container" | grep -v grep | awk '{print $1}')

  if [ -z "$host_pids" ]; then
    echo "  ⚠ No known host process found either - these nodes may need to be killed manually."
  else
    for pid in $host_pids; do
      cmd=$(ps -p "$pid" -o cmd=)
      echo "  ❌ Killing PID $pid ($cmd)"
      kill -9 "$pid"
    done
  fi
fi

echo "✅ Done."
