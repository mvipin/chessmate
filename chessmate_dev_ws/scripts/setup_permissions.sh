#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# Setup script permissions for ChessMate project

echo "🔧 Setting up ChessMate Script Permissions"
echo "=========================================="

# Get the project root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "📁 Project root: $PROJECT_ROOT"
echo "📁 Scripts directory: $SCRIPT_DIR"

# Make all shell scripts executable
echo ""
echo "🔐 Making shell scripts executable..."

# Scripts in this directory
for script in "$SCRIPT_DIR"/*.sh; do
    if [ -f "$script" ]; then
        chmod +x "$script"
        echo "✅ Made executable: $(basename "$script")"
    fi
done

# ChessBoard scripts
CHESSBOARD_DIR="$PROJECT_ROOT/ChessBoard"
if [ -d "$CHESSBOARD_DIR" ]; then
    for script in "$CHESSBOARD_DIR"/*.sh; do
        if [ -f "$script" ]; then
            chmod +x "$script"
            echo "✅ Made executable: ChessBoard/$(basename "$script")"
        fi
    done
fi

# Robot scripts
ROBOT_DIR="$PROJECT_ROOT/Robot"
if [ -d "$ROBOT_DIR" ]; then
    for script in "$ROBOT_DIR"/*.sh; do
        if [ -f "$script" ]; then
            chmod +x "$script"
            echo "✅ Made executable: Robot/$(basename "$script")"
        fi
    done
fi

# Make Python scripts executable
echo ""
echo "🐍 Making Python scripts executable..."

for dir in "$SCRIPT_DIR" "$CHESSBOARD_DIR" "$ROBOT_DIR"; do
    if [ -d "$dir" ]; then
        for script in "$dir"/*.py; do
            if [ -f "$script" ]; then
                chmod +x "$script"
                echo "✅ Made executable: $(realpath --relative-to="$PROJECT_ROOT" "$script")"
            fi
        done
    fi
done

echo ""
echo "🎉 Permission setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Run validation: python3 scripts/validate_structure.py"
echo "2. Test the system: scripts/test_chessmate.sh"
