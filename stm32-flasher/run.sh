#!/bin/sh
# Jalankan UI flasher. Buka http://127.0.0.1:8731
exec python3 "$(dirname "$0")/server.py" "$@"
