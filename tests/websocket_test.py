#!/usr/bin/env python3
"""
Test script for the WebSocket telemetry server in navi_bot/utils/websocket.py

Tests that the server correctly:
- Imports (the `websockets` package must be installed / declared in setup.py)
- Registers and deregisters clients in the handler
- Broadcasts JSON-encoded messages to all connected clients

Connection handling is tested with fake client objects, so no network or
event-loop server is required. The broadcast tests are acceptance tests:
`asyncio.wait()` cannot take bare coroutines on Python 3.11+, so broadcast
must be reworked (e.g. asyncio.gather or explicit tasks) before they pass.
"""
import asyncio
import json
import logging
import sys

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

try:
    from navi_bot.utils.websocket import WebSocketServer
    IMPORT_ERROR = None
except ImportError as e:
    WebSocketServer = None
    IMPORT_ERROR = e

MISSING_DEP_MSG = ("cannot import navi_bot.utils.websocket — the 'websockets' package "
                   "is not installed. Run `pip install websockets` and add it to "
                   "install_requires in setup.py.")


# MARK: Fakes

class FakeClient:
    """Stands in for a connected websocket: records what was sent to it."""
    def __init__(self):
        self.sent = []

    async def send(self, message):
        self.sent.append(message)


class FakeWebSocket(FakeClient):
    """A fake inbound connection: async-iterates a fixed list of messages."""
    def __init__(self, messages=None, explode_after=None):
        super().__init__()
        self._messages = list(messages or [])
        self._explode_after = explode_after

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self._explode_after is not None and self._explode_after <= 0:
            raise RuntimeError("simulated connection error")
        if self._explode_after is not None:
            self._explode_after -= 1
        if not self._messages:
            raise StopAsyncIteration
        return self._messages.pop(0)


def require_server():
    """Return a WebSocketServer or None (with the dependency error logged)."""
    if WebSocketServer is None:
        logger.error(f"  {MISSING_DEP_MSG}")
        return None
    return WebSocketServer()


# MARK: Import / Construction

def test_module_imports():
    """The websocket module must import — `websockets` belongs in setup.py."""
    passed = True
    logger.info("TEST 1: websocket module imports (dependency installed)")
    if WebSocketServer is None:
        logger.error(f"  {MISSING_DEP_MSG}")
        logger.error(f"  Import error was: {IMPORT_ERROR}")
        passed = False
    else:
        logger.info("  OK   module imported")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_construction_defaults():
    """The server must store host/port and start with no clients."""
    passed = True
    logger.info("TEST 2: construction defaults")
    server = require_server()
    if server is None:
        logger.info("FAIL")
        return False
    if server.host != 'localhost' or server.port != 8765:
        logger.warning(f"  FAIL defaults host={server.host}, port={server.port}")
        passed = False
    elif len(server.clients) != 0:
        logger.warning(f"  FAIL expected no clients, found {len(server.clients)}")
        passed = False
    else:
        logger.info("  OK   localhost:8765 with empty client set")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Broadcast

def test_broadcast_no_clients():
    """Broadcasting with no clients connected must be a harmless no-op."""
    passed = True
    logger.info("TEST 3: broadcast with no clients is a no-op")
    server = require_server()
    if server is None:
        logger.info("FAIL")
        return False
    try:
        asyncio.run(server.broadcast({'type': 'status', 'value': 'idle'}))
        logger.info("  OK   no clients, no error")
    except Exception as e:
        logger.error(f"  broadcast raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_broadcast_reaches_all_clients():
    """Broadcast must deliver the JSON-encoded message to every connected client."""
    passed = True
    logger.info("TEST 4: broadcast reaches all connected clients")
    server = require_server()
    if server is None:
        logger.info("FAIL")
        return False
    c1, c2 = FakeClient(), FakeClient()
    server.clients.add(c1)
    server.clients.add(c2)
    payload = {'type': 'pose', 'row': 4, 'col': 7}
    try:
        asyncio.run(server.broadcast(payload))
        for i, client in enumerate((c1, c2), 1):
            if len(client.sent) != 1 or json.loads(client.sent[0]) != payload:
                logger.warning(f"  FAIL client {i} received {client.sent}")
                passed = False
        if passed:
            logger.info("  OK   both clients received the JSON payload")
    except Exception as e:
        logger.error(f"  broadcast raised {type(e).__name__}: {e}")
        logger.error("  Hint: asyncio.wait() rejects bare coroutines on Python 3.11+ — "
                     "wrap sends in tasks or use asyncio.gather().")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Handler

def test_handler_register_deregister():
    """The handler must add the client on connect and remove it on disconnect."""
    passed = True
    logger.info("TEST 5: handler registers then deregisters a client")
    server = require_server()
    if server is None:
        logger.info("FAIL")
        return False
    ws = FakeWebSocket(messages=[])  # connects, sends nothing, disconnects
    try:
        asyncio.run(server.handler(ws, '/'))
    except Exception as e:
        logger.error(f"  handler raised {type(e).__name__}: {e}")
        passed = False
    if len(server.clients) != 0:
        logger.warning(f"  FAIL client still registered after disconnect: {len(server.clients)}")
        passed = False
    if passed:
        logger.info("  OK   client added and removed cleanly")
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_handler_broadcasts_received_messages():
    """Messages received by the handler must be re-broadcast to connected clients."""
    passed = True
    logger.info("TEST 6: handler re-broadcasts received messages")
    server = require_server()
    if server is None:
        logger.info("FAIL")
        return False
    payload = {'type': 'log', 'msg': 'hello'}
    ws = FakeWebSocket(messages=[json.dumps(payload)])
    try:
        asyncio.run(server.handler(ws, '/'))
        # The sender is itself a connected client, so it should get the echo
        if len(ws.sent) != 1 or json.loads(ws.sent[0]) != payload:
            logger.warning(f"  FAIL expected echo of payload, client received {ws.sent}")
            passed = False
        else:
            logger.info("  OK   received message was broadcast back out")
    except Exception as e:
        logger.error(f"  handler/broadcast raised {type(e).__name__}: {e}")
        passed = False
    logger.info("PASS" if passed else "FAIL")
    return passed


def test_handler_cleans_up_on_error():
    """A client that errors mid-stream must still be deregistered (finally block)."""
    passed = True
    logger.info("TEST 7: handler deregisters client even on connection error")
    server = require_server()
    if server is None:
        logger.info("FAIL")
        return False
    ws = FakeWebSocket(messages=['{"x": 1}', '{"x": 2}'], explode_after=1)
    try:
        asyncio.run(server.handler(ws, '/'))
    except Exception:
        pass  # the simulated error (or broadcast bug) propagating is fine here
    if len(server.clients) != 0:
        logger.warning(f"  FAIL client still registered after error: {len(server.clients)}")
        passed = False
    else:
        logger.info("  OK   client removed despite mid-stream error")
    logger.info("PASS" if passed else "FAIL")
    return passed


# MARK: Main Method

def main():
    tests = [
        test_module_imports,
        test_construction_defaults,
        test_broadcast_no_clients,
        test_broadcast_reaches_all_clients,
        test_handler_register_deregister,
        test_handler_broadcasts_received_messages,
        test_handler_cleans_up_on_error,
    ]

    args = sys.argv[1:]
    if args and args[0] == '--list':
        for i, t in enumerate(tests, 1):
            doc = (t.__doc__ or t.__name__).strip().splitlines()[0]
            print(f"TEST {i}: {doc}")
        return

    selected = tests
    if args:
        try:
            n = int(args[0])
            if not 1 <= n <= len(tests):
                raise ValueError
        except ValueError:
            logger.error(f"Invalid test selector {args[0]!r} — use 1..{len(tests)} or --list")
            sys.exit(2)
        selected = [tests[n - 1]]

    logger.info("WebSocket Server Test Suite")
    results = [t() for t in selected]

    logger.info(f"Results: {sum(results)}/{len(results)} passed")
    logger.info("All tests complete.")
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
