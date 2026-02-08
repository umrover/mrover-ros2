#!/bin/bash

set -e

# Parse command line arguments
RESET_DB=false
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --reset) RESET_DB=true ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --reset    Clear waypoints database before running tests"
            echo "  -h, --help Show this help message"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

FRONTEND_URL="http://localhost:8080"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MROVER_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TELEOP_DIR="$MROVER_ROOT/teleoperation"
FRONTEND_DIR="$TELEOP_DIR/basestation_gui/frontend"
TEST_FILE="tests/ui/basestation.spec.js"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

error() {
    echo -e "${RED}ERROR: $1${NC}" >&2
}

warn() {
    echo -e "${YELLOW}WARNING: $1${NC}"
}

info() {
    echo -e "${GREEN}$1${NC}"
}

check_node() {
    if ! command -v node &> /dev/null; then
        error "Node.js is not installed"
        echo "Install Node.js via nvm:"
        echo "  curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash"
        echo "  nvm install --lts"
        exit 1
    fi
    info "Node.js found: $(node --version)"
}

check_npx() {
    if ! command -v npx &> /dev/null; then
        error "npx is not installed (comes with npm)"
        echo "Reinstall Node.js or run: npm install -g npm"
        exit 1
    fi
}

check_playwright() {
    if ! npx playwright --version &> /dev/null; then
        error "Playwright is not installed"
        echo "Install Playwright:"
        echo "  npm install -g @playwright/test"
        echo "  npx playwright install"
        exit 1
    fi
    info "Playwright found: $(npx playwright --version)"
}

check_playwright_browsers() {
    if ! npx playwright install --dry-run &> /dev/null 2>&1; then
        warn "Playwright browsers may not be installed"
        echo "If tests fail to launch browser, run:"
        echo "  npx playwright install"
    fi
}

check_frontend_running() {
    if ! curl -s --head --fail "$FRONTEND_URL" > /dev/null 2>&1; then
        error "Frontend is not running at $FRONTEND_URL"
        echo ""
        echo "Start the frontend first:"
        echo "  cd $FRONTEND_DIR"
        echo "  npm install  # if needed"
        echo "  npm run dev -- --port 8080"
        exit 1
    fi
    info "Frontend is running at $FRONTEND_URL"
}

check_backend_running() {
    if ! curl -s --fail "$FRONTEND_URL/api/status" > /dev/null 2>&1; then
        warn "Backend API may not be running (could not reach /api/status)"
        echo ""
        echo "Start the backend:"
        echo "  cd $TELEOP_DIR/basestation_gui/backend"
        echo "  python main.py"
        echo ""
        echo "Some tests may fail without the backend."
    else
        info "Backend API is running"
    fi
}

check_test_file() {
    if [ ! -f "$TELEOP_DIR/$TEST_FILE" ]; then
        error "Test file not found: $TELEOP_DIR/$TEST_FILE"
        exit 1
    fi
    info "Test file found: $TEST_FILE"
}

echo "=========================================="
echo "  MRover Teleop Frontend Test Runner"
echo "=========================================="
echo ""

info "Checking dependencies..."
check_node
check_npx
check_playwright
check_playwright_browsers
echo ""

info "Checking test environment..."
check_test_file
check_frontend_running
check_backend_running
echo ""

info "Running Playwright tests..."
echo ""

cd "$TELEOP_DIR"
set +e
npx playwright test "$TEST_FILE" --headed --config=playwright.config.js
TEST_EXIT_CODE=$?
set -e

echo ""
if [ $TEST_EXIT_CODE -eq 0 ]; then
    info "All tests passed!"
else
    error "Some tests failed (exit code: $TEST_EXIT_CODE)"
    echo ""
    echo "View trace for debugging:"
    echo "  npx playwright show-trace test-results/*/trace.zip"
fi

exit $TEST_EXIT_CODE
