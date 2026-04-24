import asyncio, json, websockets

class WebSocketServer:
    def __init__(self, host='localhost', port=8765):
        self.host = host
        self.port = port
        self.clients = set()

    async def handler(self, websocket, path):
        self.clients.add(websocket)
        try:
            async for message in websocket:
                data = json.loads(message)
                await self.broadcast(data)
        finally:
            self.clients.remove(websocket)

    async def broadcast(self, data):
        if self.clients:  # Only send if there are clients connected
            message = json.dumps(data)
            await asyncio.wait([client.send(message) for client in self.clients])

    def start(self):
        return websockets.serve(self.handler, self.host, self.port)