#!/usr/bin/env python3
"""NavPro Mini — Graph Mission Engine, Node Catalog, and Safety Harness.

Supports visual node-based missions with conditional branching, rich on-screen
UI interactions (dynamic forms, choices, media, kiosks), API/ROS integrations,
and durable execution context.
"""

from __future__ import annotations

import ast
import asyncio
import json
import re
import time
import uuid
try:
    from .base import ApiError
except Exception:
    class ApiError(Exception):
        def __init__(self, status: int, code: str, message: str, detail: Optional[dict] = None) -> None:
            super().__init__(f"{code}: {message}")
            self.status = status
            self.code = code
            self.message = message
            self.detail = detail or {}


# ==============================================================================
# 1. Complete Node Catalog (Exported for UI Palette & MCP AI Agents)
# ==============================================================================

NODE_CATALOG = {
    "start": {
        "type": "start",
        "category": "flow",
        "title": "Mission Start",
        "description": "Entry point for the mission graph.",
        "inputs": [],
        "outputs": [{"id": "next", "label": "Next", "color": "#4CAF50"}],
        "params_schema": {},
    },
    "navigate_waypoint": {
        "type": "navigate_waypoint",
        "category": "navigation",
        "title": "Drive to Waypoint",
        "description": "Navigates to a pre-defined named waypoint.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "arrived", "label": "Arrived", "color": "#4CAF50"},
            {"id": "failed", "label": "Failed", "color": "#F44336"},
            {"id": "timeout", "label": "Timeout", "color": "#FF9800"},
        ],
        "params_schema": {
            "waypoint": {"type": "string", "required": True, "description": "Name of target waypoint"},
            "tolerance_m": {"type": "number", "default": 0.25, "description": "Goal tolerance in meters"},
            "timeout_sec": {"type": "number", "default": 180.0, "description": "Maximum drive timeout in seconds"},
        },
    },
    "navigate_coordinates": {
        "type": "navigate_coordinates",
        "category": "navigation",
        "title": "Drive to Coordinates",
        "description": "Navigates to raw map coordinates (x, y, theta).",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "arrived", "label": "Arrived", "color": "#4CAF50"},
            {"id": "failed", "label": "Failed", "color": "#F44336"},
        ],
        "params_schema": {
            "x": {"type": "number", "required": True, "description": "X coordinate in meters"},
            "y": {"type": "number", "required": True, "description": "Y coordinate in meters"},
            "theta": {"type": "number", "default": 0.0, "description": "Orientation angle in radians"},
            "timeout_sec": {"type": "number", "default": 180.0, "description": "Drive timeout in seconds"},
        },
    },
    "wait": {
        "type": "wait",
        "category": "flow",
        "title": "Pause / Timer",
        "description": "Pauses execution for a specified duration.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [{"id": "next", "label": "Next", "color": "#4CAF50"}],
        "params_schema": {
            "duration_sec": {"type": "number", "required": True, "default": 5.0, "description": "Wait duration in seconds"},
        },
    },
    "dock": {
        "type": "dock",
        "category": "navigation",
        "title": "Auto-Dock",
        "description": "Navigates to dock staging pose and docks with AprilTag.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "docked", "label": "Docked", "color": "#4CAF50"},
            {"id": "failed", "label": "Failed", "color": "#F44336"},
        ],
        "params_schema": {
            "navigate_to_staging": {"type": "boolean", "default": True, "description": "Approach dock staging point first"},
        },
    },
    "undock": {
        "type": "undock",
        "category": "navigation",
        "title": "Undock",
        "description": "Disengages from charging contacts and backs out cleanly.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "undocked", "label": "Undocked", "color": "#4CAF50"},
            {"id": "failed", "label": "Failed", "color": "#F44336"},
        ],
        "params_schema": {},
    },
    "end": {
        "type": "end",
        "category": "flow",
        "title": "Mission End",
        "description": "Explicitly terminates the mission with status, summary, and optional auto-dock.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [],  # Terminal node (no outgoing ports)
        "params_schema": {
            "status": {"type": "string", "enum": ["success", "failed", "aborted"], "default": "success"},
            "message": {"type": "string", "default": "Mission completed successfully."},
            "dock_on_end": {"type": "boolean", "default": False, "description": "Auto-dock robot after finishing"},
            "sound": {"type": "string", "default": "success_chime"},
        },
    },
    "loop": {
        "type": "loop",
        "category": "flow",
        "title": "Loop / Repeat",
        "description": "Repeats execution of the loop body for N iterations or while condition holds.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "loop_body", "label": "Loop Body", "color": "#2196F3"},
            {"id": "completed", "label": "Completed", "color": "#4CAF50"},
        ],
        "params_schema": {
            "count": {"type": "integer", "default": 3, "description": "Number of loop iterations"},
            "variable_name": {"type": "string", "default": "loop_index", "description": "Context variable storing current index (0..N-1)"},
            "condition": {"type": "string", "description": "Optional boolean condition evaluated before each iteration"},
            "max_iterations": {"type": "integer", "default": 50, "description": "Safety cap against runaway loops"},
        },
    },
    "switch_mission": {
        "type": "switch_mission",
        "category": "flow",
        "title": "Switch Mission",
        "description": "Hands off execution to another mission on the same map.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "out", "label": "When Switched", "color": "#009688"},
            {"id": "failed", "label": "If Not Found", "color": "#F44336"},
        ],
        "params_schema": {
            "target_mission_id": {"type": "string", "required": True, "description": "ID of mission to switch to"},
            "transfer_context": {"type": "boolean", "default": True, "description": "Pass current context variables to new mission"},
        },
    },
    "patrol_loop": {
        "type": "patrol_loop",
        "category": "navigation",
        "title": "Patrol Loop",
        "description": "Sequentially patrols through an ordered list of waypoints for N laps.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "completed", "label": "Completed", "color": "#4CAF50"},
            {"id": "failed", "label": "Failed", "color": "#F44336"},
            {"id": "interrupted", "label": "Interrupted", "color": "#FF9800"},
        ],
        "params_schema": {
            "waypoints": {"type": "array", "required": True, "description": "Ordered list of waypoint names"},
            "laps": {"type": "integer", "default": 1, "description": "Number of laps (0 for infinite until cancelled)"},
            "dwell_sec": {"type": "number", "default": 2.0, "description": "Wait time at each waypoint in seconds"},
        },
    },
    "battery_guard": {
        "type": "battery_guard",
        "category": "logic",
        "title": "Battery Guard",
        "description": "Checks robot battery level before proceeding.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "ok", "label": "Battery OK", "color": "#4CAF50"},
            {"id": "low_battery", "label": "Low Battery", "color": "#F44336"},
        ],
        "params_schema": {
            "min_battery_pct": {"type": "number", "default": 20.0, "description": "Minimum required battery percentage"},
            "require_charging": {"type": "boolean", "default": False, "description": "Require robot to be on charger"},
        },
    },
    "ui_interaction": {
        "type": "ui_interaction",
        "category": "hri",
        "title": "Screen UI Interaction",
        "description": "Displays an interactive form, choice modal, multimedia, or kiosk on screen.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "submitted", "label": "Submitted", "color": "#4CAF50"},
            {"id": "cancelled", "label": "Cancelled", "color": "#9E9E9E"},
            {"id": "timeout", "label": "Timeout", "color": "#FF9800"},
        ],
        "params_schema": {
            "target": {
                "type": "string",
                "enum": ["robot_screen", "operator_app", "both"],
                "default": "robot_screen",
                "description": "Where to display: robot onboard touchscreen, remote operator app, or both",
            },
            "subtype": {
                "type": "string",
                "enum": ["dynamic_form", "choice", "text_input", "media_display", "speech", "kiosk"],
                "default": "dynamic_form",
                "description": "Type of on-screen presentation",
            },
            "title": {"type": "string", "default": "Operator Action Required"},
            "message": {"type": "string", "default": ""},
            "timeout_sec": {"type": "number", "default": 60.0, "description": "Timeout waiting for human response"},
            "default_option": {"type": "string", "default": "timeout", "description": "Port to take on timeout"},
            "fields": {
                "type": "array",
                "description": "Dynamic form field definitions (for dynamic_form subtype)",
                "items_schema": {
                    "key": {"type": "string", "required": True},
                    "type": {"type": "string", "enum": ["text", "number", "select", "checkbox", "switch", "signature"]},
                    "label": {"type": "string", "required": True},
                    "required": {"type": "boolean", "default": False},
                    "options": {"type": "array", "description": "Options list for select type"},
                    "default_value": {"type": "any"},
                },
            },
            "options": {
                "type": "array",
                "description": "Button options list (for choice subtype), e.g. ['Yes', 'No', 'Retry']",
                "default": ["Yes", "No"],
            },
            "media_url": {"type": "string", "description": "Image/video URL (for media_display subtype)"},
            "speech_text": {"type": "string", "description": "Text-to-speech announcement text (for speech subtype)"},
        },
    },
    "ui_choice": {
        "type": "ui_choice",
        "category": "hri",
        "title": "Ask Choice (Buttons)",
        "description": "Displays quick action buttons or multiple-choice questions on the robot screen.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "selected", "label": "Selected", "color": "#4CAF50"},
            {"id": "cancelled", "label": "Cancelled", "color": "#9E9E9E"},
            {"id": "timeout", "label": "Timeout", "color": "#FF9800"},
        ],
        "params_schema": {
            "title": {"type": "string", "default": "Choose an Option"},
            "message": {"type": "string", "default": "Please select an option to proceed:"},
            "options": {
                "type": "array",
                "description": "Button options list, e.g. ['Yes', 'No', 'Retry']",
                "default": ["Yes", "No"],
            },
            "choices": {"type": "array", "description": "Alias for options"},
            "buttons": {"type": "array", "description": "Alias for options"},
            "timeout_sec": {"type": "number", "default": 60.0, "description": "Timeout waiting for human response"},
            "default_option": {"type": "string", "default": "timeout", "description": "Port to take on timeout"},
            "sound_alert": {"type": "boolean", "default": True},
            "speech_text": {"type": "string", "description": "Optional TTS speech prompt"},
            "output_variable": {"type": "string", "description": "Variable to store chosen option"},
            "target": {"type": "string", "enum": ["robot_screen", "operator_app", "both"], "default": "robot_screen"},
        },
    },
    "ui_notification": {
        "type": "ui_notification",
        "category": "hri",
        "title": "Show Notification",
        "description": "Displays a notification banner/card with Title, Description, and an OK button.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "confirmed", "label": "OK", "color": "#4CAF50"},
            {"id": "timeout", "label": "Timeout", "color": "#FF9800"},
        ],
        "params_schema": {
            "title": {"type": "string", "default": "Notice", "description": "Notification title"},
            "message": {"type": "string", "default": "", "description": "Notification description/details"},
            "button_text": {"type": "string", "default": "OK", "description": "Confirmation button label"},
            "timeout_sec": {"type": "number", "default": 30.0, "description": "Auto-dismiss timeout in seconds (0 for indefinite)"},
            "sound_alert": {"type": "boolean", "default": True, "description": "Play notification chime"},
            "speech_text": {"type": "string", "description": "Optional TTS speech to announce with notification"},
            "target": {"type": "string", "enum": ["robot_screen", "operator_app", "both"], "default": "robot_screen"},
        },
    },
    "ui_media": {
        "type": "ui_media",
        "category": "hri",
        "title": "Multimedia Player",
        "description": "Displays image, video, or web dashboard on the robot screen.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "completed", "label": "Completed", "color": "#4CAF50"},
            {"id": "skipped", "label": "Skipped", "color": "#9E9E9E"},
            {"id": "timeout", "label": "Timeout", "color": "#FF9800"},
        ],
        "params_schema": {
            "media_type": {"type": "string", "enum": ["image", "video", "web_url"], "default": "image"},
            "url": {"type": "string", "required": True, "description": "URL to image, video or web page"},
            "duration_sec": {"type": "number", "default": 15.0, "description": "Auto-dismiss time in seconds"},
            "show_skip": {"type": "boolean", "default": True, "description": "Allow user to tap Skip button"},
            "target": {"type": "string", "enum": ["robot_screen", "operator_app", "both"], "default": "robot_screen"},
        },
    },
    "ui_speech": {
        "type": "ui_speech",
        "category": "hri",
        "title": "Voice Announcement (TTS)",
        "description": "Speaks an audible voice message through the robot speakers.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "done", "label": "Done", "color": "#4CAF50"},
        ],
        "params_schema": {
            "text": {"type": "string", "required": True, "description": "Text to speak (supports {{context.xxx}})"},
            "voice": {"type": "string", "default": "default"},
            "wait_completion": {"type": "boolean", "default": True, "description": "Wait until speech is finished"},
        },
    },
    "condition": {
        "type": "condition",
        "category": "logic",
        "title": "Condition Branch",
        "description": "Evaluates boolean expression against context variables (form inputs, battery, API results).",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "true", "label": "True", "color": "#4CAF50"},
            {"id": "false", "label": "False", "color": "#F44336"},
        ],
        "params_schema": {
            "expression": {
                "type": "string",
                "required": True,
                "description": "e.g. form.status == 'Damaged' or system.battery_pct < 25",
            },
        },
    },
    "call_api": {
        "type": "call_api",
        "category": "integration",
        "title": "HTTP Webhook / API",
        "description": "Performs an HTTP request with templated context variables.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "success", "label": "Success (2xx)", "color": "#4CAF50"},
            {"id": "failure", "label": "Failure", "color": "#F44336"},
        ],
        "params_schema": {
            "url": {"type": "string", "required": True, "description": "HTTP or HTTPS endpoint"},
            "method": {"type": "string", "enum": ["GET", "POST", "PUT", "DELETE", "PATCH"], "default": "POST"},
            "headers": {"type": "object", "default": {}},
            "payload": {"type": "any", "description": "JSON payload (supports {{context.xxx}} templating)"},
            "timeout_sec": {"type": "number", "default": 15.0},
            "ignore_error": {"type": "boolean", "default": False},
        },
    },
    "call_service": {
        "type": "call_service",
        "category": "integration",
        "title": "ROS 2 Service Call",
        "description": "Invokes a ROS 2 service on the robot.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "success", "label": "Success", "color": "#4CAF50"},
            {"id": "failure", "label": "Failure", "color": "#F44336"},
        ],
        "params_schema": {
            "service": {"type": "string", "required": True, "description": "Service name, e.g. /camera/capture"},
            "service_type": {"type": "string", "required": True, "description": "e.g. std_srvs/srv/Trigger"},
            "request": {"type": "object", "default": {}},
            "timeout_sec": {"type": "number", "default": 15.0},
            "ignore_error": {"type": "boolean", "default": False},
        },
    },
    "call_action": {
        "type": "call_action",
        "category": "integration",
        "title": "ROS 2 Action Goal",
        "description": "Sends a goal to a ROS 2 action server.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "succeeded", "label": "Succeeded", "color": "#4CAF50"},
            {"id": "failed", "label": "Failed", "color": "#F44336"},
        ],
        "params_schema": {
            "action_name": {"type": "string", "required": True, "description": "e.g. /spin"},
            "action_type": {"type": "string", "required": True, "description": "e.g. nav2_msgs/action/Spin"},
            "goal": {"type": "object", "default": {}},
            "timeout_sec": {"type": "number", "default": 300.0},
            "ignore_error": {"type": "boolean", "default": False},
        },
    },
    "set_variable": {
        "type": "set_variable",
        "category": "logic",
        "title": "Set Variable",
        "description": "Assigns a value to a variable in the mission execution context.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [{"id": "next", "label": "Next", "color": "#4CAF50"}],
        "params_schema": {
            "key": {"type": "string", "required": True, "description": "Variable name"},
            "value": {"type": "any", "required": True, "description": "Value to assign"},
        },
    },
    "notify": {
        "type": "notify",
        "category": "hri",
        "title": "Notification / Sound / LED",
        "description": "Triggers sound, LED ring pattern, or OLED text banner.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [{"id": "next", "label": "Next", "color": "#4CAF50"}],
        "params_schema": {
            "sound": {"type": "string", "description": "Sound name or chime"},
            "led_cmd": {"type": "string", "description": "e.g. solid,0,200,40 or blink,255,0,0"},
            "oled_text": {"type": "string", "description": "Text to show on ESP32 OLED"},
        },
    },
    "publish_topic": {
        "type": "publish_topic",
        "category": "integration",
        "title": "Publish ROS 2 Topic",
        "description": "Publishes a message to a ROS 2 topic.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "success", "label": "Success", "color": "#4CAF50"},
            {"id": "failure", "label": "Failure", "color": "#F44336"}
        ],
        "params_schema": {
            "topic_name": {"type": "string", "required": True},
            "message_type": {"type": "string", "required": True},
            "payload": {"type": "object", "default": {}}
        }
    },
    "relocalize": {
        "type": "relocalize",
        "category": "hardware",
        "title": "Relocalize / Set Pose",
        "description": "Trigger global localization or dock seeding.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "done", "label": "Done", "color": "#2196F3"},
            {"id": "failed", "label": "Failed", "color": "#F44336"}
        ],
        "params_schema": {
            "mode": {"type": "string", "enum": ["global_scan", "dock_seed"], "default": "global_scan"}
        }
    },
    "jog_motion": {
        "type": "jog_motion",
        "category": "hardware",
        "title": "Jog Motion (cmd_vel)",
        "description": "Send open-loop cmd_vel for a short duration.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [
            {"id": "done", "label": "Done", "color": "#2196F3"},
            {"id": "failed", "label": "Failed", "color": "#F44336"}
        ],
        "params_schema": {
            "linear_vel": {"type": "number", "default": 0.0},
            "angular_vel": {"type": "number", "default": 0.0},
            "duration_sec": {"type": "number", "default": 1.0}
        }
    },
    "emergency_stop": {
        "type": "emergency_stop",
        "category": "hardware",
        "title": "Emergency Stop",
        "description": "Halts the robot immediately and stops navigation.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [{"id": "stopped", "label": "Stopped", "color": "#F44336"}],
        "params_schema": {
            "sound_alert": {"type": "boolean", "default": True}
        }
    },
    "cancel_navigation": {
        "type": "cancel_navigation",
        "category": "hardware",
        "title": "Cancel Navigation",
        "description": "Cancels any active navigation goal.",
        "inputs": [{"id": "in", "label": "In"}],
        "outputs": [{"id": "done", "label": "Done", "color": "#2196F3"}],
        "params_schema": {
            "halt_type": {"type": "string", "enum": ["abort_goal", "zero_vel"], "default": "abort_goal"}
        }
    }
}


# ==============================================================================
# 2. Graph Validation & Linting
# ==============================================================================

def validate_graph_mission(data: dict) -> Tuple[List[dict], List[dict], str]:
    """Validate a node-and-edge mission definition.
    
    Returns:
        (nodes_list, edges_list, entrypoint_node_id)
    """
    raw_nodes = data.get("nodes")
    raw_edges = data.get("edges", [])

    if not isinstance(raw_nodes, (list, dict)) or not raw_nodes:
        raise ApiError(400, "invalid_graph", "Mission graph must contain at least one node in 'nodes'")

    # Normalize nodes to list of dicts
    nodes: List[dict] = []
    if isinstance(raw_nodes, dict):
        for nid, nval in raw_nodes.items():
            if isinstance(nval, dict):
                node_copy = dict(nval)
                node_copy.setdefault("id", nid)
                nodes.append(node_copy)
    else:
        nodes = list(raw_nodes)

    node_ids: set[str] = set()
    entrypoint: Optional[str] = data.get("entrypoint") or data.get("entrypoint_node")

    for i, node in enumerate(nodes):
        if not isinstance(node, dict):
            raise ApiError(400, "invalid_node", f"Node at index {i} must be an object")
        nid = str(node.get("id", "")).strip()
        if not nid:
            raise ApiError(400, "invalid_node", f"Node at index {i} is missing 'id'")
        if nid in node_ids:
            raise ApiError(400, "invalid_node", f"Duplicate node id: {nid!r}")
        node_ids.add(nid)

        ntype = node.get("type")
        if not ntype or ntype not in NODE_CATALOG:
            aliases = {
                "navigate": "navigate_waypoint",
                "goto": "navigate_waypoint",
                "choice": "ui_choice",
                "form": "ui_interaction",
                "dialog": "ui_interaction",
                "speech": "ui_speech",
                "tts": "ui_speech",
                "media": "ui_media",
                "ask_choice": "ui_choice",
                "ask_info": "ui_interaction",
                "notification": "ui_notification",
                "show_notification": "ui_notification",
                "alert": "ui_notification",
            }
            if ntype in aliases:
                node["type"] = aliases[ntype]
                ntype = aliases[ntype]
            else:
                label = node.get("label") or nid
                raise ApiError(400, "invalid_node", f"Step '{label}' has unrecognized step type '{ntype}'. Please use a valid mission node.")

        # Set default position if missing
        if "position" not in node or not isinstance(node["position"], dict):
            node["position"] = {"x": 100 + (i * 250), "y": 200}

        # Track entrypoint
        if ntype == "start" and not entrypoint:
            entrypoint = nid

    if not entrypoint:
        entrypoint = nodes[0]["id"]
    elif entrypoint not in node_ids:
        raise ApiError(400, "invalid_entrypoint", f"Entrypoint node {entrypoint!r} does not exist in graph")

    # Validate edges
    if not isinstance(raw_edges, list):
        raise ApiError(400, "invalid_edges", "'edges' must be a list of connections")

    edges: List[dict] = []
    for i, edge in enumerate(raw_edges):
        if not isinstance(edge, dict):
            raise ApiError(400, "invalid_edge", f"Edge at index {i} must be an object")
        from_node = str(edge.get("from_node", "")).strip()
        to_node = str(edge.get("to_node", "")).strip()
        from_port = str(edge.get("from_port", "next")).strip().lower()
        to_port = str(edge.get("to_port", "in")).strip().lower()

        if from_node not in node_ids:
            raise ApiError(400, "invalid_edge", f"Edge {i}: source node {from_node!r} does not exist")
        if to_node not in node_ids:
            raise ApiError(400, "invalid_edge", f"Edge {i}: target node {to_node!r} does not exist")

        edge_id = edge.get("id") or f"e_{from_node}_{from_port}_{to_node}"
        edges.append({
            "id": edge_id,
            "from_node": from_node,
            "from_port": from_port,
            "to_node": to_node,
            "to_port": to_port,
        })

    # Fail-safe validation: Ensure concurrent/parallel branches do not execute conflicting motion nodes
    EXCLUSIVE_MOTION_NODES = {
        'navigate_waypoint', 'navigate_coordinates', 'navigate', 'goto',
        'dock', 'undock', 'patrol_loop', 'relocalize', 'jog_motion'
    }

    forks: dict[tuple[str, str], list[str]] = {}
    for edge in edges:
        key = (edge['from_node'], edge['from_port'])
        forks.setdefault(key, []).append(edge['to_node'])

    adj: dict[str, list[str]] = {}
    for edge in edges:
        adj.setdefault(edge['from_node'], []).append(edge['to_node'])

    nodes_dict = {n['id']: n for n in nodes}

    def _get_branch_motion_nodes(start_nid: str) -> list[str]:
        visited = set()
        queue = [start_nid]
        motions = []
        while queue:
            curr = queue.pop(0)
            if curr in visited:
                continue
            visited.add(curr)
            cnode = nodes_dict.get(curr)
            if cnode and cnode.get('type') in EXCLUSIVE_MOTION_NODES:
                label = cnode.get('label') or cnode.get('title') or curr
                motions.append(f"{cnode.get('type')} ('{label}')")
            for nxt in adj.get(curr, []):
                if nxt not in visited:
                    queue.append(nxt)
        return motions

    for (from_node, from_port), targets in forks.items():
        unique_targets = list(dict.fromkeys(targets))
        if len(unique_targets) > 1:
            motion_branches = []
            for tgt in unique_targets:
                m_nodes = _get_branch_motion_nodes(tgt)
                if m_nodes:
                    motion_branches.append((tgt, m_nodes))
            if len(motion_branches) > 1:
                conflict_details = "; ".join([f"Path via '{tgt}': {', '.join(mnodes)}" for tgt, mnodes in motion_branches])
                source_label = nodes_dict.get(from_node, {}).get('label') or from_node
                raise ApiError(
                    400,
                    "parallel_motion_conflict",
                    f"Safety Violation: Step '{source_label}' branches into parallel paths that both contain robot movement ({conflict_details}). "
                    "The robot cannot execute multiple navigation or docking actions concurrently. "
                    "Please sequence movements one after another, or run non-movement actions (such as voice announcements or screen notifications) in parallel."
                )

    return nodes, edges, entrypoint


# ==============================================================================
# 3. Safe Condition Evaluator & Variable Templating
# ==============================================================================

class _SafeConditionVisitor(ast.NodeVisitor):
    """Safely evaluates boolean expressions against context without eval()."""

    ALLOWED_NODES = (
        ast.Expression, ast.Compare, ast.BoolOp, ast.UnaryOp,
        ast.Name, ast.Attribute, ast.Constant, ast.Load,
        ast.Eq, ast.NotEq, ast.Lt, ast.LtE, ast.Gt, ast.GtE,
        ast.In, ast.NotIn, ast.And, ast.Or, ast.Not,
    )

    def __init__(self, context: dict) -> None:
        self.context = context

    def eval(self, expr: str) -> bool:
        try:
            tree = ast.parse(expr, mode="eval")
        except SyntaxError as e:
            raise ValueError(f"Invalid condition syntax: {e}")
        return bool(self.visit(tree))

    def generic_visit(self, node):
        if not isinstance(node, self.ALLOWED_NODES):
            raise ValueError(f"Disallowed expression element: {type(node).__name__}")
        return super().generic_visit(node)

    def visit_Expression(self, node):
        return self.visit(node.body)

    def visit_Constant(self, node):
        return node.value

    def visit_Name(self, node):
        val = self.context.get(node.id)
        if val is None:
            for sub in ("form", "form_data", "variables", "system"):
                if isinstance(self.context.get(sub), dict) and node.id in self.context[sub]:
                    return self.context[sub][node.id]
        return val

    def visit_Attribute(self, node):
        base = self.visit(node.value)
        if isinstance(base, dict):
            return base.get(node.attr)
        return getattr(base, node.attr, None)

    def visit_UnaryOp(self, node):
        val = self.visit(node.operand)
        if isinstance(node.op, ast.Not):
            return not val
        raise ValueError(f"Unsupported unary operator: {type(node.op).__name__}")

    def visit_BoolOp(self, node):
        if isinstance(node.op, ast.And):
            for val in node.values:
                if not self.visit(val):
                    return False
            return True
        elif isinstance(node.op, ast.Or):
            for val in node.values:
                if self.visit(val):
                    return True
            return False
        raise ValueError(f"Unsupported boolean operator: {type(node.op).__name__}")

    def visit_Compare(self, node):
        left = self.visit(node.left)
        for op, comp in zip(node.ops, node.comparators):
            right = self.visit(comp)
            res = False
            if isinstance(op, ast.Eq):
                res = (left == right)
            elif isinstance(op, ast.NotEq):
                res = (left != right)
            elif isinstance(op, ast.Lt):
                res = (left < right)
            elif isinstance(op, ast.LtE):
                res = (left <= right)
            elif isinstance(op, ast.Gt):
                res = (left > right)
            elif isinstance(op, ast.GtE):
                res = (left >= right)
            elif isinstance(op, ast.In):
                res = (left in right) if right is not None else False
            elif isinstance(op, ast.NotIn):
                res = (left not in right) if right is not None else True
            else:
                raise ValueError(f"Unsupported comparison operator: {type(op).__name__}")
            if not res:
                return False
            left = right
        return True


def _resolve_condition_expr(expression: str, context: dict) -> str:
    """Resolve {var} / {{var}} tokens in a condition expression with proper quoting.

    Unlike resolve_template_value, string values are wrapped in repr() so they
    become valid Python string literals inside the expression.  Without this,
    ``{dinner_query} == "Yes"`` resolves to ``Yes == "Yes"`` where ``Yes`` is
    an undefined Name, causing comparisons to silently fail.

    Non-string values (int, float, bool, None) are substituted as-is so
    numeric comparisons like ``{count} > 3`` still work correctly.
    """
    def _repl(match: re.Match) -> str:  # type: ignore[type-arg]
        var_name = match.group(1) or match.group(2)
        found, val = _lookup_context_path(var_name.split('.'), context)
        if not found:
            return match.group(0)  # leave token; visitor handles missing vars
        if val is None:
            return 'None'
        if isinstance(val, bool):
            return 'True' if val else 'False'
        if isinstance(val, (int, float)):
            return str(val)
        # String — wrap in repr() so it's a quoted literal in the expression
        return repr(str(val))

    return _TEMPLATE_PATTERN.sub(_repl, expression.strip())


def evaluate_condition_safely(expression: str, context: dict) -> bool:
    """Safely evaluates a boolean condition string against execution context.

    Template tokens ``{var}`` / ``{{var}}`` are resolved with proper Python
    quoting before the expression reaches the AST evaluator, so e.g.
    ``{dinner_query} == "Yes"`` becomes ``'Yes' == "Yes"`` → True.
    """
    expr = expression.strip()
    if not expr:
        return True
    expr = _resolve_condition_expr(expr, context)
    return _SafeConditionVisitor(context).eval(expr)


_TEMPLATE_PATTERN = re.compile(r"\{\{\s*([\w\.\_]+)\s*\}\}|\{\s*([\w\.\_]+)\s*\}")

def _lookup_context_path(path: list[str], context: dict) -> tuple[bool, Any]:
    if not path:
        return False, None
    first = path[0]
    rest = path[1:]

    root_obj = None
    found = False

    if first == 'context':
        root_obj = context
        found = True
    elif isinstance(context.get('variables'), dict) and first in context['variables']:
        root_obj = context['variables'][first]
        found = True
    elif isinstance(context.get('form_data'), dict) and first in context['form_data']:
        root_obj = context['form_data'][first]
        found = True
    elif isinstance(context.get('form'), dict) and first in context['form']:
        root_obj = context['form'][first]
        found = True
    elif isinstance(context.get('forms'), dict) and first in context['forms']:
        root_obj = context['forms'][first]
        found = True
    elif isinstance(context.get('api_responses'), dict) and first in context['api_responses']:
        root_obj = context['api_responses'][first]
        found = True
    elif isinstance(context.get('system'), dict) and first in context['system']:
        root_obj = context['system'][first]
        found = True
    elif first in context:
        root_obj = context[first]
        found = True

    if not found:
        return False, None

    curr = root_obj
    for p in rest:
        if isinstance(curr, dict) and p in curr:
            curr = curr[p]
        elif isinstance(curr, (list, tuple)) and p.isdigit():
            idx = int(p)
            if 0 <= idx < len(curr):
                curr = curr[idx]
            else:
                return False, None
        elif hasattr(curr, p):
            curr = getattr(curr, p)
        else:
            return False, None
    return True, curr


def resolve_template_value(template: Any, context: dict) -> Any:
    """Recursively replaces {var} and {{var}} tags with values from context.

    Supports lookup in variables, form_data, forms, api_responses, and system.
    If the template is exactly a single tag (e.g. '{count}'), preserves the native
    Python type (int, float, dict, list, bool).
    """
    if isinstance(template, str):
        cleaned = template.strip()
        single_match = _TEMPLATE_PATTERN.fullmatch(cleaned)
        if single_match:
            var_name = single_match.group(1) or single_match.group(2)
            found, val = _lookup_context_path(var_name.split('.'), context)
            if found:
                return val
            return ''

        def _repl(match):
            var_name = match.group(1) or match.group(2)
            found, val = _lookup_context_path(var_name.split('.'), context)
            if found:
                return str(val) if val is not None else ''
            return match.group(0)

        return _TEMPLATE_PATTERN.sub(_repl, template)
    elif isinstance(template, dict):
        return {k: resolve_template_value(v, context) for k, v in template.items()}
    elif isinstance(template, list):
        return [resolve_template_value(item, context) for item in template]
    return template

