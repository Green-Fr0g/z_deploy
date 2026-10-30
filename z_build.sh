#!/bin/bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    set -e
fi

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
            return 1
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
            return 1
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
        colcon build --merge-install --symlink-install --base-paths src || return
    else
        print_info "Building specific packages: $package_list"
        colcon build --merge-install --symlink-install --packages-select "${packages[@]}" || return
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
        return 0
    fi

    # Clean build artifacts
    print_info "Cleaning build artifacts..."
    rm -rf build/ cmake_build/ devel/ install/ log/ .catkin_tools/ || return

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
    echo -e "  -rd, --rd VALUE  Set ROS_DOMAIN_ID (use source)"
    echo -e "  -net, --net VALUE Set Z_NETIF network interface (use source)"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS packages"
    echo -e "  $0 package1 package2  # Build specific ROS packages"
    echo -e "  $0 -c                 # Clean all build artifacts"
    echo "  source ${SCRIPT_DIR}/z_build.sh --net lo  # Set network only; no build"
    echo "  source ${SCRIPT_DIR}/z_build.sh -rd 10    # Set ROS domain only; no build"
    echo "  Environment options without package names do not trigger a build."
    echo "  Settings apply to programs subsequently started from this terminal."
}

main() {
    local original_args=("$@")
    local packages=()
    local clean_mode=false
    local ros_domain=""
    local z_netif=""
    local environment_only=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -rd|--rd)
                if [[ $# -lt 2 || -z "$2" || "$2" == -* ]]; then
                    print_error "-rd requires a value"
                    show_usage
                    return 1
                fi
                ros_domain="$2"
                shift 2
                ;;
            -net|--net)
                if [[ $# -lt 2 || -z "$2" || "$2" == -* ]]; then
                    print_error "-net requires a value"
                    show_usage
                    return 1
                fi
                z_netif="$2"
                shift 2
                ;;
            -h|--help) show_usage; return 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; return 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace
        return $?
    fi

    if [[ -n "$ros_domain" || -n "$z_netif" ]]; then
        if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
            print_error "Environment settings require source so later programs can read them."
            printf 'Usage: source %q' "${SCRIPT_DIR}/z_build.sh"
            printf ' %q' "${original_args[@]}"
            printf '\n'
            return 1
        fi
        if [[ ${#packages[@]} -eq 0 ]]; then
            environment_only=true
        fi
    fi

    # Require ROS only when a build is requested, before changing the environment.
    if [[ "$environment_only" = false && -z "$ROS_DISTRO" ]]; then
        print_error "ROS environment not detected. Please source your ROS setup.bash first."
        return 1
    fi

    if [ -n "$ros_domain" ]; then
        export ROS_DOMAIN_ID="$ros_domain"
        echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    fi
    if [ -n "$z_netif" ]; then
        export Z_NETIF="$z_netif"
        echo "Z_NETIF=${Z_NETIF}"
    fi

    if [[ "$environment_only" = true ]]; then
        return 0
    fi

    setup_inference_runtime || return
    setup_mujoco || return
    setup_gazebo_models || return
    run_ros_build "${packages[@]}"
}

main "$@"
