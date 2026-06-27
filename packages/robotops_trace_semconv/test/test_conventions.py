# Copyright 2025 Robot Ops Inc.
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

"""Smoke test: the Python mirror is importable and the key strings are exact."""

import robotops_trace_semconv as semconv


def test_concept_keys_have_expected_strings():
    """robot.* concept keys carry the exact dotted strings from the spec."""
    assert semconv.ROBOT_ACTION_GOAL_ID == 'robot.action.goal_id'
    assert semconv.ROBOT_ACTION_RESULT == 'robot.action.result'
    assert semconv.ROBOT_ACTION_NAME == 'robot.action.name'
    assert semconv.ROBOT_CALLBACK_TYPE == 'robot.callback.type'


def test_ros_mapping_keys_have_expected_strings():
    """ros.* mapping keys carry the exact dotted strings from the spec."""
    assert semconv.ROS_TOPIC == 'ros.topic'
    assert semconv.ROS_PUBLISHER_GID == 'ros.publisher_gid'
    assert semconv.ROS_SOURCE_TIMESTAMP == 'ros.source_timestamp'


def test_resource_keys_reuse_otel_and_robot_id():
    """Resource attributes reuse OTel's service.name and add robot.id."""
    assert semconv.SERVICE_NAME == 'service.name'
    assert semconv.ROBOT_ID == 'robot.id'


def test_enum_values_are_lowercase_literals():
    """Enumerated string values match the dictionary."""
    assert semconv.ROBOT_ACTION_RESULT_SUCCEEDED == 'succeeded'
    assert semconv.ROBOT_ACTION_RESULT_ABORTED == 'aborted'
    assert semconv.ROBOT_ACTION_RESULT_CANCELED == 'canceled'
    assert semconv.ROBOT_CALLBACK_TYPE_SUBSCRIPTION == 'subscription'
