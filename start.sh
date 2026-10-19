#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/web"
exec python3 server.py --open "$@"
