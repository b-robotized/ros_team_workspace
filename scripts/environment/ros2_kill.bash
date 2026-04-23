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

echo "🔍 Listing ROS 2 nodes..."
nodes=$(ros2 node list)

if [ -z "$nodes" ]; then
  echo "✅ No ROS 2 nodes are currently running."
  exit 0
fi

echo "🧠 Resolving PIDs of ROS 2 nodes..."
for node in $nodes; do
  echo "➡ Node: $node"
  pids=$(ps -eo pid,cmd | grep "$node" | grep -v grep | awk '{print $1}')
  if [ -z "$pids" ]; then
    echo "  ⚠ No PID found for $node"
  else
    for pid in $pids; do
      echo "  ❌ Killing PID $pid"
      kill -9 $pid
    done
  fi
done

echo "✅ Done."
