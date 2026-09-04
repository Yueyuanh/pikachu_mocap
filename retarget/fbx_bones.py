"""
retarget/fbx_bones.py — 尽力而为地从 FBX 二进制读取骨骼名(离线兜底)。

用途: 当 Pikachu_Retarget.py 未连上 Blender 时, 从皮肤的 FBX 源文件读骨骼名,
      填充直接操控的骨列表。连上 Blender 后, request_scene 实时返回的骨架骨骼
      始终优先(见 Pikachu_Retarget.py 的 _on_scene)。再次兜底是 skin map 里
      手工填写的 bones 列表。

实现说明:
  FBX 二进制属性区没有"7-bit 紧凑长度"编码。S(字符串)类型是
  [1字节类型码 'S'][4字节 uint 字符串长度][字节串]; R(raw)/数组类型
  前有 [4字节 count][4字节 encoding=0/1][4字节 compressed_len]。
  Model 节点的 Properties 固定为 [0]=id(Li64), [1]=name(S), [2]=class(S)，
  仅当 class ∈ {LimbNode, Root, Joint, Null} 时视为骨骼。用记录头
  end_offset 做硬边界, 避免坏文件越界; 另加最大记录数/深度保护。

返回: {"bone_name": parent_name 或 None, ...}(层序近似)。解析失败回空。
"""

import struct


class _FBXReader:
    __slots__ = ("f", "count", "limit")

    def __init__(self, path, limit=4_000_000):
        self.f = open(path, "rb")
        self.count = 0
        self.limit = limit

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass

    @staticmethod
    def _get_u8(f):
        b = f.read(1)
        return b[0] if b else None

    @staticmethod
    def _get_u16(f):
        b = f.read(2)
        return struct.unpack("<H", b)[0] if len(b) == 2 else None

    @staticmethod
    def _get_u32(f):
        b = f.read(4)
        return struct.unpack("<I", b)[0] if len(b) == 4 else None

    @staticmethod
    def _get_i64(f):
        b = f.read(8)
        return struct.unpack("<q", b)[0] if len(b) == 8 else None

    def parse_props(self, f, end):
        """解析属性区到列表, 绝不越过 end。返回属性列表。"""
        out = []
        while f.tell() < end:
            t = self._get_u8(f)
            if t is None:
                break
            t = chr(t)
            if t in "C":  # bool 1字节
                f.read(1)
            elif t == "Y":  # int16
                f.read(2)
            elif t in "IF":  # int32 / float32
                f.read(4)
            elif t in "DL":  # float64 / int64
                f.read(8)
            elif t in "SR":  # 字符串 / raw
                n = self._get_u32(f)
                if n is None or n > 128 * 1024:  # 名称最长也就几十字节
                    break
                data = f.read(n)
                if len(data) < n:
                    break
                out.append(data)
            elif t in "biID lfL".replace(" ", "") or (t and t in "bd"):  # 数组类型
                # 数组: [count][encoding][compressed_len]+(count*es)
                cnt = self._get_u32(f)
                enc = self._get_u32(f)
                clen = self._get_u32(f)
                if cnt is None or cnt > 1_000_000:
                    break
                es = {"b": 1, "i": 4, "l": 8, "f": 4, "d": 8}.get(t, 4)
                size = clen if (enc == 1 and clen) else (cnt * es)
                if size > 64 * 1024 * 1024:
                    break
                f.read(size)
            # 其余未知类型直接忽略该字节并继续
            if f.tell() > end:
                break
        return out

    def walk_models(self):
        """遍历全部顶层记录, 返回 [(name, props)] 及 Connections 的 OO。"""
        models = []      # [(oid, name, class)]
        conns = []       # [(child, parent)] 来自 OO
        self._walk(0, models, conns)
        return models, conns

    def _walk(self, depth, models, conns):
        if depth > 32 or self.count > self.limit:
            return
        while True:
            start = self.f.tell()
            head = self.f.read(13)
            if len(head) < 13:
                return
            end_rel, np_, plen = struct.unpack("<III", head[:12])
            nlen = head[12]
            if nlen > 255:
                return
            name = self.f.read(nlen).decode("latin1", "ignore")
            # name 后 4 字节对齐(相对记录起始)
            pad = (4 - ((13 + nlen) % 4)) % 4
            self.f.seek(self.f.tell() + pad)
            prop_end = self.f.tell() + plen
            if plen:
                props = self.parse_props(self.f, prop_end)
            else:
                props = []
            if self.f.tell() != prop_end:
                self.f.seek(prop_end)
            # 4 对齐到记录区尾部
            abs_end = start + end_rel
            if name == "Model" and len(props) >= 3:
                oid, nm, cls = props[0], props[1], props[2]
                if not isinstance(nm, bytes):
                    nm = b""
                if not isinstance(cls, bytes):
                    cls = b""
                models.append((oid, nm.decode("utf-8", "ignore"),
                               cls.decode("utf-8", "ignore")))
            elif name == "Connections" and not conns:
                # 直接读子记录里的 OO 类型连接
                self._read_conns(prop_end, abs_end, conns)

            # 进入子记录区
            sub_start = self.f.tell()
            if abs_end > sub_start:
                self._walk(depth + 1, models, conns)
            self.f.seek(abs_end)
            self.count += 1
            if abs_end <= start:
                return

    def _read_conns(self, prop_end, abs_end, conns):
        """在 Connections 段的子记录区找 c(child),o(parent) 对。"""
        self.f.seek(prop_end)
        pos = self.f.tell()
        guard = 0
        while pos < abs_end and guard < 200000:
            head = self.f.read(13)
            if len(head) < 13:
                return
            end_rel, np_, plen = struct.unpack("<III", head[:12])
            nlen = head[12]
            cname = self.f.read(nlen).decode("latin1", "ignore")
            pad = (4 - ((13 + nlen) % 4)) % 4
            self.f.seek(self.f.tell() + pad)
            prop_end2 = self.f.tell() + plen
            if plen:
                props = self.parse_props(self.f, prop_end2)
            else:
                props = []
            self.f.seek(prop_end2)
            abs_end2 = pos + end_rel
            if cname in ("OO",) and len(props) >= 4:
                # 对节点 id 解释: 属性里 id 是 resonate 的数值;
                # 此处 props[0]=child_id, props[1]=parent_id(为 int)
                def _num(x):
                    return x if isinstance(x, int) else None
                ch, pa = props[0], props[1]
                if isinstance(ch, bytes):
                    ch = _num(props[0])
                conns.append((ch, pa))
            pos = abs_end2
            self.f.seek(pos)
            guard += 1


def fbx_bone_names(path):
    """从 FBX 读骨骼名 dict {bone: parent}。失败回空。"""
    r = _FBXReader(path)
    try:
        magic = r.f.read(23)
        if not magic.startswith(b"Kaydara FBX Binary"):
            return {}
        r.f.read(4)  # 版本
        models, conns = r.walk_models()
        id2name = {}
        id2cls = {}
        for oid, nm, cls in models:
            if not isinstance(oid, int):
                continue
            if cls not in ("LimbNode", "Root", "Joint", "Null"):
                continue
            if not nm:
                continue
            id2name[oid] = nm
            id2cls[oid] = cls
        parent = {}
        for ch, pa in conns:
            if ch in id2name and pa in id2name:
                parent[id2name[ch]] = id2name[pa]
        # 未配对的根骨
        for oid, nm in id2name.items():
            if nm not in parent:
                parent[nm] = None
        if not parent:
            return {}
        return parent
    except Exception:
        return {}
    finally:
        r.close()


if __name__ == "__main__":
    import sys
    p = sys.argv[1]
    print("bones:", fbx_bone_names(p))