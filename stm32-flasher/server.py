#!/usr/bin/env python3
"""stm32-flasher — UI sederhana untuk build & upload firmware ke STM32 lewat ST-LINK.

Jalankan:  python3 server.py      lalu buka http://127.0.0.1:8731
Butuh:     arm-none-eabi-gcc, stlink  (brew install arm-none-eabi-gcc stlink)
"""

import http.server
import json
import os
import re
import shutil
import socketserver
import subprocess
import threading
import time
import webbrowser
from collections import deque
from pathlib import Path
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
UPLOADS = ROOT / "uploads"
HOST = "127.0.0.1"
PORT = int(os.environ.get("PORT", "8731"))

DEFAULT_PROJECT = REPO / "stm32-pd"
FLASH_BASE = "0x08000000"

# Perintah make yang boleh dijalankan di folder proyek. Tidak ada shell,
# tidak ada string dari browser yang masuk ke argumen perintah.
MAKE_ACTIONS = {
    "build":   (["make"],           "Build"),
    "flash":   (["make", "flash"],  "Build & Flash"),
    "backup":  (["make", "backup"], "Backup flash board"),
    "restore": (["make", "restore"], "Restore firmware lama"),
    "clean":   (["make", "clean"],  "Clean"),
}

HEX_RE = re.compile(r"^0x[0-9a-fA-F]{1,8}$")
SAFE_NAME_RE = re.compile(r"[^A-Za-z0-9._-]")


# ---------------------------------------------------------------- log buffer

class Log:
    """Baris log dengan nomor urut monoton, dipoll oleh UI."""

    def __init__(self, cap=4000):
        self.lines = deque(maxlen=cap)
        self.seq = 0
        self.lock = threading.Lock()

    def add(self, text, kind="out"):
        with self.lock:
            self.lines.append({"i": self.seq, "t": text, "k": kind})
            self.seq += 1

    def since(self, n):
        with self.lock:
            return [l for l in self.lines if l["i"] >= n], self.seq


LOG = Log()


# ---------------------------------------------------------------- job runner

class Job:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False
        self.label = ""
        self.rc = None
        self.finished_at = 0.0

    def snapshot(self):
        with self.lock:
            return {"running": self.running, "label": self.label, "rc": self.rc}


JOB = Job()


def _stream(proc):
    """Baca stdout mentah, pecah di \\n maupun \\r supaya progress bar tetap terbaca."""
    buf = b""
    while True:
        chunk = os.read(proc.stdout.fileno(), 4096)
        if not chunk:
            break
        buf += chunk
        while True:
            idx = min((i for i in (buf.find(b"\n"), buf.find(b"\r")) if i >= 0), default=-1)
            if idx < 0:
                break
            line, buf = buf[:idx], buf[idx + 1:]
            LOG.add(line.decode("utf-8", "replace").rstrip())
    if buf:
        LOG.add(buf.decode("utf-8", "replace").rstrip())


def start_job(cmd, cwd, label):
    """Jalankan satu perintah di thread terpisah. Hanya satu job boleh jalan."""
    with JOB.lock:
        if JOB.running:
            return False, "Masih ada proses berjalan"
        JOB.running, JOB.label, JOB.rc = True, label, None

    def worker():
        LOG.add("", "blank")
        LOG.add(f"$ {' '.join(cmd)}    ({cwd})", "cmd")
        rc = -1
        try:
            proc = subprocess.Popen(
                cmd, cwd=str(cwd),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                env={**os.environ, "PATH": os.environ.get("PATH", "")},
            )
            _stream(proc)
            rc = proc.wait()
        except FileNotFoundError:
            LOG.add(f"perintah tidak ditemukan: {cmd[0]}", "err")
        except Exception as exc:                        # noqa: BLE001
            LOG.add(f"gagal menjalankan: {exc}", "err")
        LOG.add(f"{label} selesai (exit {rc})", "ok" if rc == 0 else "err")
        with JOB.lock:
            JOB.running, JOB.rc, JOB.finished_at = False, rc, time.time()

    threading.Thread(target=worker, daemon=True).start()
    return True, label


# ---------------------------------------------------------------- probe board

class Probe:
    """Cache hasil `st-info --probe`; ST-LINK tidak boleh diganggu saat flashing."""

    def __init__(self):
        self.data = {"connected": False, "note": "belum diperiksa"}
        self.lock = threading.Lock()

    def get(self):
        with self.lock:
            return dict(self.data)

    def refresh(self):
        if not shutil.which("st-info"):
            info = {"connected": False, "note": "st-info tidak ada — brew install stlink"}
        else:
            try:
                out = subprocess.run(["st-info", "--probe"], capture_output=True,
                                     text=True, timeout=6).stdout
            except Exception:                            # noqa: BLE001
                out = ""
            fields = dict(re.findall(r"^\s*([a-z-]+):\s+(.+?)\s*$", out, re.M))
            found = re.search(r"Found (\d+) stlink", out)
            n = int(found.group(1)) if found else 0
            if n and fields:
                flash = fields.get("flash", "")
                size = re.match(r"(\d+)", flash)
                info = {
                    "connected": True,
                    "chip": fields.get("dev-type", "?"),
                    "chipid": fields.get("chipid", ""),
                    "serial": fields.get("serial", ""),
                    "version": fields.get("version", ""),
                    "flash": int(size.group(1)) if size else 0,
                    "sram": int(re.match(r"(\d+)", fields.get("sram", "0")).group(1)),
                }
            else:
                info = {"connected": False, "note": "board tidak terdeteksi"}
        with self.lock:
            self.data = info


PROBE = Probe()


def probe_loop():
    while True:
        if not JOB.snapshot()["running"]:
            PROBE.refresh()
        time.sleep(2.5)


# ---------------------------------------------------------------- http server

def project_info(path_str):
    p = Path(path_str).expanduser()
    if not p.is_absolute():
        p = (REPO / p).resolve()
    info = {"path": str(p), "exists": p.is_dir(), "makefile": False, "bins": []}
    if info["exists"]:
        info["makefile"] = (p / "Makefile").is_file()
        for b in sorted(p.rglob("*.bin")):
            if ".git" in b.parts:
                continue
            info["bins"].append({
                "path": str(b),
                "name": str(b.relative_to(p)),
                "size": b.stat().st_size,
                "mtime": b.stat().st_mtime,
            })
        info["bins"] = info["bins"][:20]
    return info


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    # -- helpers
    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path, ctype):
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    # -- routes
    def do_GET(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)

        if url.path in ("/", "/index.html"):
            return self.send_file(ROOT / "index.html", "text/html; charset=utf-8")

        if url.path == "/api/poll":
            since = int(q.get("since", ["0"])[0])
            lines, seq = LOG.since(since)
            return self.send_json({
                "lines": lines, "seq": seq,
                "job": JOB.snapshot(), "device": PROBE.get(),
            })

        if url.path == "/api/project":
            return self.send_json(project_info(q.get("path", [str(DEFAULT_PROJECT)])[0]))

        if url.path == "/api/env":
            return self.send_json({
                "repo": str(REPO),
                "default_project": str(DEFAULT_PROJECT),
                "tools": {t: bool(shutil.which(t)) for t in
                          ("make", "arm-none-eabi-gcc", "st-flash", "st-info")},
            })

        self.send_error(404)

    def do_POST(self):
        url = urlparse(self.path)
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b""

        if url.path == "/api/upload":
            q = parse_qs(url.query)
            name = SAFE_NAME_RE.sub("_", q.get("name", ["firmware.bin"])[0])[-64:]
            UPLOADS.mkdir(exist_ok=True)
            dest = UPLOADS / name
            dest.write_bytes(raw)
            LOG.add(f"file diterima: {dest}  ({len(raw)} byte)", "ok")
            return self.send_json({"ok": True, "path": str(dest), "size": len(raw)})

        if url.path == "/api/run":
            try:
                req = json.loads(raw or b"{}")
            except ValueError:
                return self.send_json({"ok": False, "error": "body bukan JSON"}, 400)
            return self.send_json(*self.dispatch(req))

        self.send_error(404)

    def dispatch(self, req):
        action = req.get("action", "")

        if action in MAKE_ACTIONS:
            info = project_info(req.get("project", str(DEFAULT_PROJECT)))
            if not info["exists"]:
                return {"ok": False, "error": f"folder tidak ada: {info['path']}"}, 400
            if not info["makefile"]:
                return {"ok": False, "error": "tidak ada Makefile di folder itu"}, 400
            cmd, label = MAKE_ACTIONS[action]
            ok, msg = start_job(cmd, Path(info["path"]), label)
            return {"ok": ok, "error": None if ok else msg}, 200 if ok else 409

        if action == "flash_file":
            path = Path(req.get("file", "")).expanduser()
            addr = req.get("addr", FLASH_BASE)
            if not path.is_file():
                return {"ok": False, "error": f"file tidak ada: {path}"}, 400
            if not HEX_RE.match(addr):
                return {"ok": False, "error": "alamat harus hex, mis. 0x08000000"}, 400
            ok, msg = start_job(["st-flash", "--reset", "write", str(path), addr],
                                REPO, f"Flash {path.name} → {addr}")
            return {"ok": ok, "error": None if ok else msg}, 200 if ok else 409

        if action == "read_flash":
            info = project_info(req.get("project", str(DEFAULT_PROJECT)))
            size = PROBE.get().get("flash") or 0x100000
            out = Path(info["path"]) / "backup" / "board-dump.bin"
            out.parent.mkdir(parents=True, exist_ok=True)
            ok, msg = start_job(["st-flash", "read", str(out), FLASH_BASE, hex(size)],
                                REPO, f"Baca flash → {out.name}")
            return {"ok": ok, "error": None if ok else msg}, 200 if ok else 409

        if action == "erase":
            ok, msg = start_job(["st-flash", "erase"], REPO, "Erase seluruh flash")
            return {"ok": ok, "error": None if ok else msg}, 200 if ok else 409

        return {"ok": False, "error": f"aksi tidak dikenal: {action}"}, 400


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    threading.Thread(target=probe_loop, daemon=True).start()
    missing = [t for t in ("make", "arm-none-eabi-gcc", "st-flash", "st-info")
               if not shutil.which(t)]
    LOG.add("stm32-flasher siap.", "ok")
    if missing:
        LOG.add("tool belum terpasang: " + ", ".join(missing), "err")
    url = f"http://{HOST}:{PORT}"
    print(f"stm32-flasher berjalan di {url}  (Ctrl+C untuk berhenti)")
    threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        Server((HOST, PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        print("\nberhenti.")


if __name__ == "__main__":
    main()
