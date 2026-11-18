#!/bin/bash
# ESP Terminal for Orakul 5.2
# Usage: ./esp-term.sh
# Requires: curl, python3

IP="${ORAKUL_IP:-192.168.3.15}"
PIN="${ORAKUL_PIN:-ovsyanka_12449512}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# History file
HISTFILE="$HOME/.esp-term-history"
HISTSIZE=500
HISTFILESIZE=500

# Load previous history if file exists
if [ -f "$HISTFILE" ]; then
    history -r "$HISTFILE"
fi

# Check if ESP is alive
check_alive() {
    if ! curl -s --noproxy "*" --max-time 3 "http://$IP/exec?pin=$PIN&cmd=status" > /dev/null 2>&1; then
        echo -e "${RED}ERROR: ESP not reachable at $IP${NC}"
        echo -e "${YELLOW}Check:${NC}"
        echo -e "  - ESP powered on?"
        echo -e "  - Connected to Wi-Fi?"
        echo -e "  - IP correct? (current: $IP)"
        echo -e "${YELLOW}Try:${NC} ping $IP"
        exit 1
    fi
}

# URL-encode command
urlencode() {
    python3 -c "import urllib.parse,sys; print(urllib.parse.quote(sys.argv[1]))" "$1"
}

# Send command to ESP
send_cmd() {
    local cmd="$1"
    local encoded
    encoded=$(urlencode "$cmd")
    local response
    response=$(curl -s --noproxy "*" --max-time 10 "http://$IP/exec?pin=$PIN&cmd=$encoded")
    if [ -z "$response" ]; then
        echo -e "${RED}ERROR: Empty response (timeout?)${NC}"
    else
        echo "$response"
    fi
}

# Print header
print_header() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  Orakul 5.2 ESP Terminal${NC}"
    echo -e "${CYAN}  IP:   $IP${NC}"
    echo -e "${CYAN}  Введите 'help' для списка команд${NC}"
    echo -e "${CYAN}  Введите 'exit' для выхода${NC}"
    echo -e "${CYAN}  Стрелки ↑/↓ — история команд${NC}"
    echo -e "${CYAN}========================================${NC}"
}

# ===== MAIN =====
check_alive
print_header

while true; do
    # read -e включает readline: стрелки, история, редактирование строки
    # -p задаёт промпт на каждой новой строке
    if ! read -e -p "$(echo -e "${GREEN}esp@8266> ${NC}")" cmd; then
        # Ctrl-D / EOF
        echo -e "${YELLOW}Bye!${NC}"
        break
    fi

    # Add to in-memory history (readline уже добавил, но подстрахуемся для пустых)
    if [ -n "$cmd" ]; then
        history -s "$cmd"
    fi

    # Empty input → status
    if [ -z "$cmd" ]; then
        cmd="status"
    fi

    # Exit
    if [ "$cmd" = "exit" ] || [ "$cmd" = "quit" ]; then
        echo -e "${YELLOW}Bye!${NC}"
        break
    fi

    # Help
    if [ "$cmd" = "help" ] || [ "$cmd" = "?" ]; then
        echo -e "${CYAN}Команды:${NC}"
        echo -e "  status       - краткий статус"
        echo -e "  time         - время, дата"
        echo -e "  weather      - погода"
        echo -e "  home         - домашняя погода"
        echo -e "  sun          - рассвет/закат"
        echo -e "  moon         - луна"
        echo -e "  advice       - советчик"
        echo -e "  users        - юзеры"
        echo -e "  log          - лог FLASH"
        echo -e "  heap         - RAM"
        echo -e "  wifi         - Wi-Fi"
        echo -e "  scan         - сканировать Wi-Fi"
        echo -e "  gpio all     - все GPIO"
        echo -e "  reboot       - перезагрузить"
        echo -e "  reset        - сбросить EEPROM"
        echo -e "  exit         - выход"
        echo -e "\n  Linux-подобные:"
        echo -e "  ls           - файлы"
        echo -e "  ps           - процессы"
        echo -e "  free         - RAM"
        echo -e "  df           - flash"
        echo -e "  uname -a     - версия"
        echo -e "  date         - дата"
        echo -e "  ifconfig     - сеть"
        echo -e "  top          - CPU/RAM"
        echo -e "  dmesg        - лог загрузки"
        continue
    fi

    # Send
    send_cmd "$cmd"
    echo
done

# Save history to file on exit
history -w "$HISTFILE"
