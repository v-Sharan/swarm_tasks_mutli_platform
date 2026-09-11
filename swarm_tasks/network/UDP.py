import json
import socket
import socketserver
import threading

class CallBack:
    def __init__(self):
        self.data = None
        
    def call_back(self,data,addr):
        self.data = data.decode()
        
    def get_current_data(self):
        return self.data
    
    def set_data(self):
        self.data = None

class UDPReceiverHandler(socketserver.BaseRequestHandler):
    """Handles each incoming UDP packet by delegating to the server's callback."""

    def handle(self):
        data, sock = self.request
        # Delegate to whatever callback the server was configured with
        self.server.on_data_received(data, self.client_address)


class UDPReceiver(socketserver.ThreadingMixIn, socketserver.UDPServer):
    """
    A UDP server dedicated to receiving data.
    """

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, callback=None, buffer_size=65536):
        super().__init__(address, UDPReceiverHandler)
        self.buffer_size = buffer_size
        self._callback = callback or self._default_callback
        self._serve_thread = None

    def _default_callback(self, data, addr):
        print(f"[UDPReceiver] {addr} -> {data.decode(errors='replace')}")

    def on_data_received(self, data: bytes, addr):
        """Called by the handler for every packet. Override or pass callback= instead."""
        print(data)
        self._callback(data, addr)

    def start(self):
        """Run the server loop in a background thread (non-blocking)."""
        self._serve_thread = threading.Thread(target=self.serve_forever, daemon=True)
        self._serve_thread.start()
        return self

    def stop(self):
        self.shutdown()
        self.server_close()
        if self._serve_thread:
            self._serve_thread.join()


class UDPSender:
    """Fire-and-forget UDP transmitter.

    One socket, opened once and reused. The destination address is passed
    on every send() rather than fixed at construction, so callers can
    point it at a runtime-editable host/port (e.g. Variables.terrain_warn_*)
    without rebuilding anything.
    """

    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, data, host, port):
        if isinstance(data, str):
            data = data.encode()
        self._sock.sendto(data, (host, int(port)))

    def send_json(self, obj, host, port):
        self.send(json.dumps(obj), host, port)

    def close(self):
        self._sock.close()
