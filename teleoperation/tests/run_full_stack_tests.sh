#!/bin/bash

# Full-stack test runner for teleoperation
# Runs both Playwright (frontend) and pytest (backend/ROS) tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(dirname "$SCRIPT_DIR")"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_services() {
    print_status "Checking required services..."

    if ! curl -s http://localhost:8000/docs > /dev/null 2>&1; then
        print_error "Backend server not running on localhost:8000"
        print_warning "Start with: cd $TELEOP_DIR/basestation_gui && ./gui_backend.sh"
        return 1
    fi
    print_status "Backend server: OK"

    if ! curl -s http://localhost:8080 > /dev/null 2>&1; then
        print_error "Frontend server not running on localhost:8080"
        print_warning "Start with: cd $TELEOP_DIR/basestation_gui/frontend && npm run dev"
        return 1
    fi
    print_status "Frontend server: OK"

    return 0
}

run_playwright_tests() {
    print_status "Running Playwright tests..."
    cd "$TELEOP_DIR"

    if [ ! -d "node_modules" ]; then
        print_warning "Installing Playwright dependencies..."
        npm install
        npx playwright install
    fi

    npx playwright test tests/ui/ --reporter=list
}

run_pytest_integration() {
    print_status "Running pytest integration tests..."
    cd "$SCRIPT_DIR"

python3 -m pytest integration/ -v --tb=short
}

run_all() {
    print_status "Running full-stack tests for teleoperation"
    echo ""

    if ! check_services; then
        print_error "Required services not running. Aborting."
        exit 1
    fi

    echo ""
    run_playwright_tests
    echo ""
    run_pytest_integration
    echo ""

    print_status "All tests completed!"
}

case "${1:-all}" in
    playwright)
        check_services && run_playwright_tests
        ;;
    pytest)
        check_services && run_pytest_integration
        ;;
    check)
        check_services
        ;;
    all)
        run_all
        ;;
    *)
        echo "Usage: $0 {all|playwright|pytest|check}"
        echo ""
        echo "Commands:"
        echo "  all        Run all tests (default)"
        echo "  playwright Run only Playwright UI tests"
        echo "  pytest     Run only pytest integration tests"
        echo "  check      Check if required services are running"
        exit 1
        ;;
esac
