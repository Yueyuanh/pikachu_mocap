import json
import queue
import socket
import threading
import time

HOST = "0.0.0.0"
PORT = 9999
BUFFER_SIZE = 4096

msg_queue = queue.Queue()
log_messages = []
_log_lock = threading.Lock()
LOG_LIMIT = 50

server_running = False
client_connected = False
state_dirty = False

client_conn = None
server_socket = None
server_thread = None
stop_event = threading.Event()


def _mark_state_dirty():
    global state_dirty
    state_dirty = True


def add_log(message):
    text = str(message)
    stamp = time.strftime("%H:%M:%S")
    line = f"[{stamp}] {text}"
    with _log_lock:
        log_messages.append(line)
        if len(log_messages) > LOG_LIMIT:
            del log_messages[:-LOG_LIMIT]
    _mark_state_dirty()


def _set_server_running(running):
    global server_running
    if server_running != running:
        server_running = running
        _mark_state_dirty()
        if running:
            add_log("Server running (waiting for client)")
        else:
            add_log("Server stopped")


def _set_client_state(connected, conn=None):
    global client_connected
    global client_conn
    if client_connected != connected or client_conn is not conn:
        client_connected = connected
        client_conn = conn if connected else None
        _mark_state_dirty()
        if connected:
            add_log("Client connected")
        else:
            add_log("Client disconnected")


def _close_socket(sock):
    if sock is None:
        return
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass


def start_server():
    global server_thread

    if server_running:
        return

    stop_event.clear()

    server_thread = threading.Thread(
        target=socket_server,
        daemon=True
    )

    server_thread.start()

    print("Server thread started")
    add_log("Server thread started")


def stop_server():
    stop_event.set()

    _close_socket(client_conn)
    _set_client_state(False, None)

    if server_socket is not None:
        try:
            server_socket.close()
        except Exception:
            pass

    _set_server_running(False)

    print("Server stopped")


def send_message(payload):
    conn = client_conn
    if not client_connected or conn is None:
        return False

    try:
        data = json.dumps(payload) + "\n"
        conn.sendall(data.encode())
        return True
    except Exception:
        _close_socket(conn)
        _set_client_state(False, None)
        return False


def socket_server():
    global server_socket

    try:
        server_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        server_socket.settimeout(0.5)

    except Exception as e:
        print("Server error:", e)
        add_log(f"Server error: {e}")
        _set_server_running(False)
        return

    _set_server_running(True)

    print("Listening on port", PORT)
    add_log(f"Listening on port {PORT}")

    while not stop_event.is_set():

        try:
            conn, addr = server_socket.accept()
        except socket.timeout:
            continue
        except Exception:
            break

        print("Client connected:", addr)
        add_log(f"Client connected: {addr}")

        # 新客户端接入时先断开旧的（服务端只保留一个客户端，避免旧连接占坑导致无响应）
        old = client_conn
        if old is not None and old is not conn:
            _close_socket(old)

        conn.settimeout(0.5)
        _set_client_state(True, conn)

        buffer = ""

        try:
            while not stop_event.is_set():
                try:
                    data = conn.recv(BUFFER_SIZE)
                except socket.timeout:
                    continue

                if not data:
                    break

                buffer += data.decode(errors="replace")

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        msg_queue.put(line)

        except Exception:
            pass

        _set_client_state(False, None)
        _close_socket(conn)

        print("Client disconnected")
        add_log("Client disconnected")

    _set_client_state(False, None)
    _set_server_running(False)

    if server_socket is not None:
        try:
            server_socket.close()
        except Exception:
            pass
        server_socket = None
