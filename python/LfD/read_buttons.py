import threading
import json as json_module
import ssl
import typing
import requests 
from websockets.sync.client import connect

class DeskEventListener:
    def __init__(self, hostname: str, username: str, password: str):
        self._session = requests.Session()
        self._session.verify = False
        self._hostname = hostname
        self._username = username
        self._password = password
        self._listening = False
        self._listen_thread = None

    def _listen(self, cb, timeout):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        with connect(
            f'wss://{self._hostname}/desk/api/navigation/events',
            ssl_context=ctx,
            additional_headers={
                'authorization': self._session.cookies.get('authorization')
            }) as websocket:
            self._listening = True
            while self._listening:
                try:
                    event: typing.Dict = json_module.loads(websocket.recv(timeout))
                    cb(event)
                except TimeoutError:
                    pass

    def start_listening(self, cb: typing.Callable[[typing.Dict], None]) -> None:
        self._listen_thread = threading.Thread(target=self._listen, args=(cb, 1.0))
        self._listen_thread.start()

    def stop_listening(self) -> None:
        self._listening = False
        if self._listen_thread is not None:
            self._listen_thread.join()

def event_callback(event: typing.Dict) -> None:
    print("Received event:", event)

if __name__ == "__main__":
    hostname = "172.16.0.2"
    username = "your_username"
    password = "your_password"

    listener = DeskEventListener(hostname, username, password)
    listener.start_listening(event_callback)

    input("Press Enter to stop listening...")
    
    listener.stop_listening()
