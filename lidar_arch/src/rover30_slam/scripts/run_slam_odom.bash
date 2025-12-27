#!/bin/bash
set -e

# =========================
# Skrypt do sprawdzania paczek ROS2, czyszczenia procesów i uruchomienia launch
# =========================

ROS_DISTRO=${ROS_DISTRO:-humble}

# --- Funkcje pomocnicze ---

# Sprawdzenie, czy komenda istnieje
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Sprawdzenie, czy paczka ROS2 istnieje
ros2_package_exists() {
    ros2 pkg list 2>/dev/null | grep -q "^$1\$"
}

# Kolory logów
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status()  { echo -e "${YELLOW}[STATUS]${NC} $1"; }
print_success() { echo -e "${GREEN}[OK]${NC} $1"; }
print_warning() { echo -e "${RED}[WARN]${NC} $1"; }
print_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }

# --- Funkcja cleanup ---

cleanup_processes() {
    print_status "Czyszczenie istniejących procesów ROS2 związanych z launch..."
    
    # Twoje węzły w launch
    pkill -f "odom_from_joint_states" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "map_saver_server" 2>/dev/null || true
    pkill -f "lifecycle_manager_slam" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true

    sleep 2
    print_success "Procesy oczyszczone"
}

# --- Sprawdzenie ROS2 ---
if ! command_exists ros2; then
    print_warning "ROS2 nie jest zainstalowany lub nie w PATH"
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    print_warning "ROS_DISTRO nie jest ustawiony. Sproś źródło setup.bash ROS2"
    exit 1
fi

print_success "ROS2 $ROS_DISTRO wykryty"

# --- Sprawdzenie wymaganych paczek ---
REQUIRED_ROS_PACKAGES=(
    "slam_toolbox"
    "nav2_map_server"
    "nav2_lifecycle_manager"
)

MISSING_ROS_PACKAGES=()
for pkg in "${REQUIRED_ROS_PACKAGES[@]}"; do
    print_status "Sprawdzam paczkę ROS2: $pkg"
    if ros2_package_exists "$pkg"; then
        print_success "Paczkę $pkg znaleziono"
    else
        print_warning "Paczkę $pkg brak"
        MISSING_ROS_PACKAGES+=("$pkg")
    fi
done

# Instalacja brakujących paczek
if [ ${#MISSING_ROS_PACKAGES[@]} -gt 0 ]; then
    print_info "Instalacja brakujących paczek..."
    sudo apt update
    for pkg in "${MISSING_ROS_PACKAGES[@]}"; do
        print_info "Instaluję paczkę: $pkg"
        sudo apt install -y ros-${ROS_DISTRO}-$pkg
        print_success "Zainstalowano paczkę: $pkg"
    done
else
    print_success "Wszystkie wymagane paczki są zainstalowane"
fi

# --- Cleanup starych procesów ---
cleanup_processes

# --- Uruchomienie launch ---
print_info "Uruchamiam launch file: odom_slam.launch.py"
ros2 launch rover30_slam odom_slam.launch.py
