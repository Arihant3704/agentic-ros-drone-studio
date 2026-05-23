#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define colours for logging
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0;30m' # No Color
RESET='\033[0m'

echo -e "${CYAN}=====================================================${RESET}"
echo -e "${CYAN}    Agentic ROS Drone Simulation Studio - Launcher   ${RESET}"
echo -e "${CYAN}=====================================================${RESET}"

# Change directory to script folder
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

# Verify python dependencies
echo -e "${GREEN}[*] Verifying python environment...${RESET}"
python3 -c "import fastapi, uvicorn, websockets, requests, jinja2" 2>/dev/null || {
    echo -e "${RED}[!] Missing python packages. Attempting to install...${RESET}"
    pip3 install fastapi uvicorn websockets requests jinja2
}

# Verify if Ollama is running and has the model
echo -e "${GREEN}[*] Checking Ollama service...${RESET}"
if curl -s http://localhost:11434/api/tags > /dev/null; then
    echo -e "${GREEN}[*] Ollama service is running.${RESET}"
    # Check if qwen3.5:0.8b is pulled
    if curl -s http://localhost:11434/api/tags | grep -q "qwen3.5:0.8b"; then
        echo -e "${GREEN}[*] Local model qwen3.5:0.8b is available. AI Agent will run locally.${RESET}"
    else
        echo -e "${YELLOW}[!] Model qwen3.5:0.8b not found in Ollama list.${RESET}"
        echo -e "${YELLOW}[!] The AI Agent will use the high-fidelity simulator fallback engine for instant feedback.${RESET}"
    fi
else
    echo -e "${YELLOW}[!] Ollama service is not running on http://localhost:11434.${RESET}"
    echo -e "${YELLOW}[!] Please start Ollama or pull 'qwen3.5:0.8b' if you wish to run physical local LLM calls.${RESET}"
    echo -e "${YELLOW}[!] The simulator will automatically use its internal rule-based ReAct agent fallback.${RESET}"
fi

# Define Port
PORT=8000
echo -e "${GREEN}[*] Launching FastAPI ROS Bridge on port ${PORT}...${RESET}"

# Start FastAPI server in the background
python3 app.py &
SERVER_PID=$!

# Function to clean up on exit
cleanup() {
    echo -e "\n${YELLOW}[*] Shutting down simulation server (PID: ${SERVER_PID})...${RESET}"
    kill $SERVER_PID 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Give server a second to start
sleep 1.5

# Open browser if possible
URL="http://localhost:${PORT}"
echo -e "${GREEN}[*] Simulation running at: ${CYAN}${URL}${RESET}"
echo -e "${GREEN}[*] Press Ctrl+C to stop the simulation server.${RESET}"

# Attempt to open web browser
if which xdg-open > /dev/null; then
    xdg-open "$URL" > /dev/null 2>&1 &
elif which gnome-open > /dev/null; then
    gnome-open "$URL" > /dev/null 2>&1 &
elif python3 -m webbrowser "$URL" > /dev/null 2>&1; then
    true
else
    echo -e "${YELLOW}[!] Could not automatically open the browser. Please open: ${CYAN}${URL}${RESET}"
fi

# Keep script running and wait for background server to finish
wait $SERVER_PID
