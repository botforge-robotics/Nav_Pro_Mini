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
            # Check for legacy alias mapping
            if ntype == "navigate":
                node["type"] = "navigate_waypoint"
                ntype = "navigate_waypoint"
            else:
                raise ApiError(400, "invalid_node", f"Node {nid!r} has unknown type {ntype!r}")

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


def evaluate_condition_safely(expression: str, context: dict) -> bool:
    """Safely evaluates a boolean condition string against execution context."""
    expr = expression.strip()
    if not expr:
        return True
    return _SafeConditionVisitor(context).eval(expr)


_TEMPLATE_PATTERN = re.compile(r"\{\{\s*([\w\.\_]+)\s*\}\}")

def resolve_template_value(template: Any, context: dict) -> Any:
    """Recursively replaces {{context.key}} tags with values from context."""
    if isinstance(template, str):
        def _repl(match):
            path = match.group(1).split(".")
            curr: Any = context
            for p in path:
                if isinstance(curr, dict) and p in curr:
                    curr = curr[p]
                elif p == "context":
                    continue
                else:
                    return ""
            return str(curr) if curr is not None else ""
        return _TEMPLATE_PATTERN.sub(_repl, template)
    elif isinstance(template, dict):
        return {k: resolve_template_value(v, context) for k, v in template.items()}
    elif isinstance(template, list):
        return [resolve_template_value(item, context) for item in template]
    return template
