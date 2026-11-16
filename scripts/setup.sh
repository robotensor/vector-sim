#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../js"
npm ci --no-audit --no-fund
