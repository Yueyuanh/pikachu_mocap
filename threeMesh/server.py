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


if __name__ == '__main__':
    print(f'threeMesh preview  →  http://127.0.0.1:{PORT}/  (Ctrl-C 退出)')
    with ThreadedTCPServer(('', PORT), Handler) as httpd:
        httpd.serve_forever()