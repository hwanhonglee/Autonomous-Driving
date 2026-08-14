# Copyright 2026 hwanhonglee
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

# HH_260814 - Static regression gate for the no-actuation VILS launch composition.
if(NOT DEFINED LAUNCH_FILE OR NOT EXISTS "${LAUNCH_FILE}")
  message(FATAL_ERROR "RX-only launch file is missing: ${LAUNCH_FILE}")
endif()
if(NOT DEFINED RECEIVER_LAUNCH_FILE OR NOT EXISTS "${RECEIVER_LAUNCH_FILE}")
  message(FATAL_ERROR "Receiver child launch file is missing: ${RECEIVER_LAUNCH_FILE}")
endif()
if(NOT DEFINED RUNNER_FILE OR NOT EXISTS "${RUNNER_FILE}")
  message(FATAL_ERROR "RX-only runner file is missing: ${RUNNER_FILE}")
endif()

file(READ "${LAUNCH_FILE}" launch_contents)
string(FIND "${launch_contents}" "socket_can_receiver.launch.py" receiver_position)
if(receiver_position EQUAL -1)
  message(FATAL_ERROR "RX-only launch does not include the SocketCAN receiver")
endif()
string(REGEX MATCHALL "socket_can_receiver\\.launch\\.py" receiver_includes "${launch_contents}")
list(LENGTH receiver_includes receiver_include_count)
if(NOT receiver_include_count EQUAL 1)
  message(FATAL_ERROR
    "RX-only launch must contain exactly one receiver include; found ${receiver_include_count}")
endif()

foreach(forbidden_text
    "socket_can_sender"
    "twistController2VCU2EPS2ACC"
    "twistController2vcu"
    "twistController2EPS2ACC"
    "can_brdige.launch.xml"
    "socket_can_bridge.launch.xml")
  string(FIND "${launch_contents}" "${forbidden_text}" forbidden_position)
  if(NOT forbidden_position EQUAL -1)
    message(FATAL_ERROR "RX-only launch contains forbidden action: ${forbidden_text}")
  endif()
endforeach()

# HH_260814 - The child launch is pre-existing code; pin it so a future sender include cannot bypass
# the wrapper-only text check without an explicit safety review and runner hash update.
file(SHA256 "${RECEIVER_LAUNCH_FILE}" receiver_launch_sha256)
set(approved_receiver_launch_sha256
  "aa43a8bcf25a7450b38d6f003b42da04d80c55987dc84202b9f3f4a78e4aefee")
if(NOT "${receiver_launch_sha256}" STREQUAL "${approved_receiver_launch_sha256}")
  message(FATAL_ERROR
    "Receiver child launch hash changed: ${receiver_launch_sha256}; safety review required")
endif()

file(READ "${RUNNER_FILE}" runner_contents)
foreach(required_runner_text
    "LISTEN-ONLY"
    "launch_provenance_guard_reason"
    "publisher_guard_reason"
    "process_starttime"
    "abort_is_latched"
    "cyclonedds_profile_guard_reason"
    "dedicated_session_group_state"
    "final_prelaunch_guard_reason"
    "receiver_mapped_library_guard_reason"
    "stop_unregistered_direct_child"
    "vils_approved_receiver_binary_sha256"
    "vils_approved_receiver_component_sha256"
    "vils_approved_receiver_core_sha256"
    "can_fleet_guard_reason"
    "stable_tx_endpoint_guard_reason"
    "tx_endpoint_guard_reason"
    "trap '' TSTP")
  string(FIND "${runner_contents}" "${required_runner_text}" runner_position)
  if(runner_position EQUAL -1)
    message(FATAL_ERROR "RX-only runner lacks required safety gate: ${required_runner_text}")
  endif()
endforeach()
