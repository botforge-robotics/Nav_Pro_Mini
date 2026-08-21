#!/usr/bin/env python3
"""Shared request handling: JSON contract, errors, auth, staleness."""

from __future__ import annotations

import json
import math
from typing import Any, Optional

import tornado.web


class ApiError(Exception):
    """Raised by handlers; turned into the standard error body."""

    def __init__(self, status: int, code: str, message: str,
                 detail: Optional[dict] = None) -> None:
        super().__init__(message)
        self.status = status
        self.code = code
        self.message = message
        self.detail = detail or {}


class BaseHandler(tornado.web.RequestHandler):
    """Every SDK endpoint inherits this.

    One error shape everywhere:
        {"error": {"code": "...", "message": "...", "detail": {...}}}
    paired with a real HTTP status. A client can branch on `code` without
    parsing prose, and prose can be improved without breaking clients.
    """

    def initialize(self, bridge=None, **kw) -> None:  # noqa: D401
        self.bridge = bridge
        self.opts = kw

    # CORS so browser dashboards can call the robot directly. The robot is a
    # LAN device with no cookie-based session, so there is no CSRF surface to
    # protect here; token auth (when enabled) travels in a header, which
    # browsers will not attach cross-origin by accident.
    def set_default_headers(self) -> None:
        self.set_header('Content-Type', 'application/json')
        self.set_header('Access-Control-Allow-Origin', '*')
        self.set_header('Access-Control-Allow-Headers', 'Content-Type, Authorization')
        self.set_header('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS')

    def options(self, *_args, **_kw) -> None:
        self.set_status(204)
        self.finish()

    def prepare(self) -> None:
        token = self.opts.get('auth_token')
        if not token or self.request.method == 'OPTIONS':
            return
        sent = self.request.headers.get('Authorization', '')
        if sent != f'Bearer {token}':
            self.fail(401, 'unauthorized', 'Missing or invalid bearer token')

    # -- responses ---------------------------------------------------------

    def send(self, payload: Any, status: int = 200) -> None:
        self.set_status(status)
        self.finish(json.dumps(self._clean(payload)))

    def fail(self, status: int, code: str, message: str,
             detail: Optional[dict] = None) -> None:
        self.set_status(status)
        self.finish(json.dumps({'error': {
            'code': code, 'message': message, 'detail': detail or {}}}))

    def write_error(self, status_code: int, **kwargs) -> None:
        exc = kwargs.get('exc_info', (None, None, None))[1]
        if isinstance(exc, ApiError):
            self.fail(exc.status, exc.code, exc.message, exc.detail)
            return
        self.set_status(status_code)
        self.finish(json.dumps({'error': {
            'code': 'internal_error',
            'message': str(exc) if exc else 'Unhandled server error',
            'detail': {}}}))

    @staticmethod
    def _clean(obj: Any) -> Any:
        """JSON has no NaN/Infinity. Sensor floats do.

        Emitting bare NaN produces output that strict parsers reject, so
        non-finite values become null — "no reading" rather than a number that
        cannot be represented.
        """
        if isinstance(obj, float):
            return obj if math.isfinite(obj) else None
        if isinstance(obj, dict):
            return {k: BaseHandler._clean(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [BaseHandler._clean(v) for v in obj]
        return obj

    # -- requests ----------------------------------------------------------

    def body(self, required: tuple[str, ...] = ()) -> dict:
        raw = (self.request.body or b'').decode('utf-8').strip()
        data = {}
        if raw:
            try:
                data = json.loads(raw)
            except ValueError as exc:
                raise ApiError(400, 'invalid_json', f'Body is not valid JSON: {exc}')
        if not isinstance(data, dict):
            raise ApiError(400, 'invalid_body', 'Body must be a JSON object')
        missing = [f for f in required if f not in data]
        if missing:
            raise ApiError(400, 'missing_field',
                           f'Missing required field(s): {", ".join(missing)}',
                           {'missing': missing})
        return data

    def cached(self, key: str, name: str) -> dict:
        """Read a telemetry snapshot, with its age attached.

        404 when the topic has never produced data: that is a different
        condition from "the value is zero", and a caller debugging a dead
        sensor needs to be able to tell them apart.
        """
        value, age = self.bridge.get_with_age(key)
        if value is None:
            raise ApiError(503, 'no_data',
                           f'No {name} data received yet — is the robot fully started?',
                           {'source': key})
        return {'data': value, 'age_sec': age}
