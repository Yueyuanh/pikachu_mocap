#!/usr/bin/env python3
"""threeMesh 本地预览服务器。

纯 ES 模块(ESM)页面不能被 file:// 直接打开(CORS 会把模块加载挡掉),
所以用这个极简 http 服务器把 threeMesh/ 目录吐出来。
"""
import http.server
import os
import socketserver

PORT = 8767
HERE = os.path.dirname(os.path.abspath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=HERE, **kw)

    def end_headers(self):
        # 每次改动 assets/lib 都别被浏览器缓存坑到
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, *a):
        pass


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    # 8767 被占就自动往后找空闲端口，避免报 Address already in use
    for port in range(PORT, PORT + 2000):
        try:
            httpd = ThreadedTCPServer(('', port), Handler)
        except OSError:
            continue
        break
    else:
        print('找不到空闲端口'); return 1
    print(f'threeMesh preview  →  http://127.0.0.1:{port}/  (Ctrl-C 退出)')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())