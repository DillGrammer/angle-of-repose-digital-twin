#!/bin/zsh
set -e

cd "${0:A:h}"

PYTHON_FOR_SIM="/opt/homebrew/bin/python3.13"
if [[ ! -x "$PYTHON_FOR_SIM" ]]; then
  PYTHON_FOR_SIM="$(command -v python3.13 || true)"
fi

if [[ -z "$PYTHON_FOR_SIM" ]]; then
  echo "Python 3.13 is required for the Apple Silicon PyBullet package."
  echo "Install Python 3.13, then open this launcher again."
  read -k 1 "?Press any key to close."
  exit 1
fi

if [[ ! -d ".venv" ]]; then
  "$PYTHON_FOR_SIM" -m venv .venv
fi

source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python main.py
