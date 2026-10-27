#!/bin/bash
set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Load common utilities
source "${SCRIPT_DIR}/scripts/common.sh"

# ========================
# Configuration
# ========================

# ========================
# Build Functions
# ========================

setup_inference_runtime() {
    print_header "[Setting up Inference Runtime]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_SCRIPT="${SCRIPT_DIR}/scripts/download_inference_runtime.sh"

    if [ -f "$DOWNLOAD_SCRIPT" ]; then
        print_info "Checking inference libraries..."
        bash "$DOWNLOAD_SCRIPT" || {
            print_error "Failed to setup inference libraries"
            exit 1
        }
        print_success "Inference runtime setup completed!"
    else
        print_warning "Download script not found: $DOWNLOAD_SCRIPT"
    fi
}

setup_mujoco() {
    print_header "[Setting up MuJoCo]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_MUJOCO_SCRIPT="${SCRIPT_DIR}/scripts/download_mujoco.sh"

    if [ -f "$DOWNLOAD_MUJOCO_SCRIPT" ]; then
        print_info "Checking MuJoCo library..."
        bash "$DOWNLOAD_MUJOCO_SCRIPT" || {
            print_error "Failed to setup MuJoCo"
            exit 1
        }
        print_success "MuJoCo setup completed!"
    else
        print_warning "MuJoCo download script not found: $DOWNLOAD_MUJOCO_SCRIPT"
    fi
}

setup_gazebo_models() {
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_GAZEBO_MODELS_SCRIPT="${SCRIPT_DIR}/scripts/download_gazebo_models.sh"

    if [ -f "$DOWNLOAD_GAZEBO_MODELS_SCRIPT" ]; then
        bash "$DOWNLOAD_GAZEBO_MODELS_SCRIPT" || {
            print_warning "Gazebo models setup skipped (optional for simulation)"
        }
    fi
}

run_ros_build() {
    local packages=("$@")
    local package_list=$(IFS=' '; echo "${packages[*]}")

    print_header "[Running ROS Build]"

    # Execute build
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Building all packages..."
        colcon build --merge-install --symlink-install --base-paths src
    else
        print_info "Building specific packages: $package_list"
        colcon build --merge-install --symlink-install --packages-select $package_list
    fi

    print_success "ROS build completed!"
}

# ========================
# Clean Functions
# ========================

clean_workspace() {
    print_header "[Cleaning Workspace]"

    # Show what will be cleaned
    print_info "The following will be cleaned:"
    echo "  - directory build/"
    echo "  - directory cmake_build/"
    echo "  - directory devel/"
    echo "  - directory install/"
    echo "  - directory log/"
    echo "  - directory .catkin_tools/"

    # Ask for confirmation
    if ! ask_confirmation "Are you sure you want to clean all build artifacts?"; then
        print_warning "Clean operation cancelled."
        exit 0
    fi

    # Clean build artifacts
    print_info "Cleaning build artifacts..."
    rm -rf build/ cmake_build/ devel/ install/ log/ .catkin_tools/

    print_success "Clean completed!"
}

# ========================
# Main Script
# ========================

show_usage() {
    print_header "[Build System Usage]"
    print_header
    echo -e "Usage: $0 [OPTIONS] [PACKAGE_NAMES...]"
    echo ""
    echo -e "${COLOR_INFO}Options:${COLOR_RESET}"
    echo -e "  -c, --clean      Clean workspace (remove build artifacts)"
    echo -e "  -rd VALUE        Set ROS_DOMAIN_ID"
    echo -e "  -net VALUE       Set Z_NETIF network interface"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS packages"
    echo -e "  $0 package1 package2  # Build specific ROS packages"
    echo -e "  $0 -c                 # Clean all build artifacts"
}

main() {
    local packages=()
    local clean_mode=false
    local ros_domain=""
    local z_netif=""

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -rd)
                if [[ $# -lt 2 || -z "$2" ]]; then
                    print_error "-rd requires a value"
                    show_usage
                    exit 1
                fi
                ros_domain="$2"
                shift 2
                ;;
            -net)
                if [[ $# -lt 2 || -z "$2" ]]; then
                    print_error "-net requires a value"
                    show_usage
                    exit 1
                fi
                z_netif="$2"
                shift 2
                ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace
        exit 0
    fi

    # Handle ROS build
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS environment not detected. Please source your ROS setup.bash first."
        exit 1
    fi

    if [ -n "$ros_domain" ]; then
        export ROS_DOMAIN_ID="$ros_domain"
        echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    fi
    if [ -n "$z_netif" ]; then
        export Z_NETIF="$z_netif"
        echo "Z_NETIF=${Z_NETIF}"
    fi

    setup_inference_runtime
    setup_mujoco
    setup_gazebo_models
    run_ros_build "${packages[@]}"
}

main "$@"
