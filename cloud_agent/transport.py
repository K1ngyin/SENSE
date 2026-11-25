"""Single-robot HTTPS pull transport for the cloud Agent. Python 3.10+."""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import json
import math
import socket
import ssl
import threading
import time
import uuid
from collections import OrderedDict
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


MAX_BODY = 32 * 1024
MAX_CONNECTIONS = 12
REQUEST_TIMEOUT = 5
ACK_HISTORY = 128


def _json_bytes(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode()


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _receipt(result):
    return hashlib.sha256(json.dumps(result, ensure_ascii=False, allow_nan=False,
                                     sort_keys=True, separators=(",", ":")).encode()).digest()


@dataclass
class _Request:
    command: dict
    deadline: float
    delivered: bool = False
    result: dict | None = None


class Broker:
    """Offer each request to at most one poll, even when its HTTP response is lost."""

    def __init__(self, timeout=15.0):
        if not math.isfinite(timeout) or not 0 < timeout <= 300:
            raise ValueError("timeout must be positive and at most 300 seconds")
        self.timeout = timeout
        self.address = None
        self._changed = threading.Condition()
        self._pending = None
        self._history = OrderedDict()
        self._closed = False

    def _finish(self, item, result, *, from_gateway=False):
        item.result = result
        self._history[item.command["request_id"]] = _receipt(result) if from_gateway else None
        if len(self._history) > ACK_HISTORY:
            self._history.popitem(last=False)
        self._pending = None
        self._changed.notify_all()

    def _end_locally(self, status, reason):
        item = self._pending
        self._finish(item, {"status": status, "data": {
            "reason": reason, "request_id": item.command["request_id"],
            "execution_unknown": item.delivered,
        }})

    def _expire(self):
        if self._pending and time.monotonic() >= self._pending.deadline:
            self._end_locally("TIMEOUT", "Gateway acknowledgement deadline passed")

    def call(self, tool_name, arguments):
        """Submit one already validated high-level tool and wait for its acknowledgement."""
        if not isinstance(tool_name, str) or not tool_name or not isinstance(arguments, dict):
            raise ValueError("invalid tool name or arguments")
        with self._changed:
            if self._closed:
                return {"status": "CANCELLED", "data": {"reason": "broker closed"}}
            self._expire()
            if self._pending:
                if tool_name != "stop_robot" or self._pending.command["tool"] == "stop_robot":
                    return {"status": "BLOCKED", "data": {"reason": "another call is pending"}}
            expires_at = datetime.fromtimestamp(time.time() + self.timeout, timezone.utc)
            command = {"request_id": str(uuid.uuid4()), "tool": tool_name,
                       "arguments": arguments, "expires_at": expires_at.isoformat()}
            wire = _json_bytes({"command": command})
            if len(wire) > MAX_BODY:
                raise ValueError("command exceeds size limit")
            if self._pending:
                self._end_locally("CANCELLED", "superseded by stop_robot")
            item = _Request(json.loads(wire)["command"], time.monotonic() + self.timeout)
            self._pending = item
            while item.result is None:
                self._changed.wait(max(0, item.deadline - time.monotonic()))
                self._expire()
            return item.result

    def poll(self):
        with self._changed:
            self._expire()
            item = self._pending
            if item is None or item.delivered:
                return {"command": None}
            item.delivered = True
            return {"command": item.command}

    def receive(self, message):
        if set(message) != {"request_id", "status", "data"}:
            return 400
        request_id, status, data = message["request_id"], message["status"], message["data"]
        if not isinstance(request_id, str) or len(request_id) != 36:
            return 400
        if not isinstance(status, str) or not status or len(status) > 64 or not isinstance(data, dict):
            return 400
        result = {"status": status, "data": data}
        with self._changed:
            self._expire()
            if request_id in self._history:
                previous = self._history[request_id]
                if previous is None:
                    return 410
                return 200 if previous == _receipt(result) else 409
            item = self._pending
            if item is None or item.command["request_id"] != request_id:
                return 404
            if not item.delivered:
                return 409
            self._finish(item, result, from_gateway=True)
            return 200

    def close(self):
        with self._changed:
            self._closed = True
            if self._pending:
                self._end_locally("CANCELLED", "broker closed")


class _GatewayHTTP(ThreadingHTTPServer):
    def process_request(self, request, address):
        if not self.capacity.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except Exception:
            self.capacity.release()
            raise

    def process_request_thread(self, request, address):
        def interrupt():
            try:
                request.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

        deadline = threading.Timer(REQUEST_TIMEOUT, interrupt)
        try:
            request.settimeout(REQUEST_TIMEOUT)
            if self.tls:
                request = self.tls.wrap_socket(request, server_side=True, do_handshake_on_connect=False)
            deadline.start()
            if self.tls:
                request.do_handshake()
            self.finish_request(request, address)
        except (OSError, TimeoutError):
            pass
        finally:
            deadline.cancel()
            self.shutdown_request(request)
            self.capacity.release()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Avoid request-body, credential or untrusted path logging.

    def _reply(self, code, body):
        wire = _json_bytes(body)
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(wire)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(wire)

    def do_POST(self):
        auth = self.headers.get_all("Authorization", [])
        if len(auth) != 1 or not hmac.compare_digest(auth[0].encode(), self.server.bearer):
            return self._reply(401, {"error": "unauthorized"})
        if self.path not in ("/v1/poll", "/v1/result"):
            return self._reply(404, {"error": "unknown endpoint"})
        if self.headers.get_content_type() != "application/json":
            return self._reply(415, {"error": "use application/json"})
        lengths = self.headers.get_all("Content-Length", [])
        if self.headers.get("Transfer-Encoding") or len(lengths) != 1:
            return self._reply(400, {"error": "one Content-Length required"})
        if not lengths[0].isascii() or not lengths[0].isdigit() or len(lengths[0]) > 8:
            return self._reply(400, {"error": "invalid content length"})
        size = int(lengths[0])
        if size > MAX_BODY:
            return self._reply(413, {"error": "body too large"})
        try:
            raw = self.rfile.read(size)
            body = json.loads(raw.decode("utf-8"), object_pairs_hook=_object)
            if len(raw) != size or not isinstance(body, dict):
                raise ValueError("invalid object")
            _json_bytes(body)  # Reject non-finite numbers and unpaired surrogates.
        except (ValueError, RecursionError):
            return self._reply(400, {"error": "invalid JSON object"})
        if self.path == "/v1/poll":
            if body:
                return self._reply(400, {"error": "poll expects an empty object"})
            return self._reply(200, self.server.broker.poll())
        code = self.server.broker.receive(body)
        return self._reply(code, {"accepted": code == 200})


@contextmanager
def serve_gateway(token, *, host="127.0.0.1", port=8787, certfile=None, keyfile=None, timeout=15.0):
    """Start the endpoint while the caller runs its interactive Agent in this process."""
    if not isinstance(token, str) or not token.isascii() or not token.isprintable() or not token.strip() or " " in token:
        raise ValueError("provide an ASCII bearer token without spaces")
    if bool(certfile) != bool(keyfile):
        raise ValueError("TLS requires both certfile and keyfile")
    ip = ipaddress.ip_address("127.0.0.1" if host == "localhost" else host)
    if not certfile and not ip.is_loopback:
        raise ValueError("TLS required outside loopback; bind loopback behind a local HTTPS reverse proxy")
    tls = None
    if certfile:
        tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        tls.minimum_version = ssl.TLSVersion.TLSv1_2
        tls.load_cert_chain(certfile, keyfile)
    broker = Broker(timeout)

    class Server(_GatewayHTTP):
        address_family = socket.AF_INET6 if ip.version == 6 else socket.AF_INET

    with Server((str(ip), port), _Handler) as server:
        server.broker = broker
        server.tls = tls
        server.bearer = ("Bearer " + token).encode()
        server.capacity = threading.BoundedSemaphore(MAX_CONNECTIONS)
        broker.address = server.server_address[:2]
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        thread.start()
        try:
            yield broker
        finally:
            broker.close()
            server.shutdown()
            thread.join()
