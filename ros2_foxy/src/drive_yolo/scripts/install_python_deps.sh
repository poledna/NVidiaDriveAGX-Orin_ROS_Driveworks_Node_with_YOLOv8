#!/bin/bash

# install_python_deps.sh
# Install Python dependencies for drive_yolo 3D tracking

echo "🎯 Installing Python dependencies for drive_yolo 3D tracking..."

# Check if we're on DRIVE AGX Orin (aarch64)
ARCH=$(uname -m)
echo "Architecture: $ARCH"

# Update pip first
echo "📦 Updating pip..."
python3 -m pip install --upgrade pip

# Install core dependencies
echo "📦 Installing core dependencies..."
pip3 install numpy>=1.19.0 scipy>=1.5.0

# Install motpy (primary choice)
echo "🎯 Installing motpy for advanced tracking..."
if pip3 install motpy>=0.0.10; then
    echo "✅ motpy installed successfully"
else
    echo "⚠️  motpy installation failed, will use custom tracker"
fi

# Install scikit-learn for clustering
echo "🧠 Installing scikit-learn for point clustering..."
if pip3 install scikit-learn>=0.24.0; then
    echo "✅ scikit-learn installed successfully"
else
    echo "⚠️  scikit-learn installation failed, clustering disabled"
fi

# Install filterpy as backup
echo "📊 Installing filterpy as backup Kalman filter..."
if pip3 install filterpy>=1.4.5; then
    echo "✅ filterpy installed successfully"
else
    echo "⚠️  filterpy installation failed, advanced filtering disabled"
fi

# Test imports
echo "🧪 Testing Python imports..."
python3 -c "
import sys
success = True

try:
    import numpy as np
    print('✅ numpy:', np.__version__)
except ImportError as e:
    print('❌ numpy failed:', e)
    success = False

try:
    import scipy
    print('✅ scipy:', scipy.__version__)
except ImportError as e:
    print('❌ scipy failed:', e)
    success = False

try:
    import motpy
    print('✅ motpy: available')
except ImportError:
    print('⚠️  motpy: not available (will use custom tracker)')

try:
    import sklearn
    print('✅ sklearn:', sklearn.__version__)
except ImportError:
    print('⚠️  sklearn: not available (clustering disabled)')

try:
    import filterpy
    print('✅ filterpy: available')
except ImportError:
    print('⚠️  filterpy: not available (advanced Kalman disabled)')

if success:
    print('\\n✅ Core dependencies satisfied! Python 3D tracking ready.')
    sys.exit(0)
else:
    print('\\n❌ Some core dependencies failed. Check installation.')
    sys.exit(1)
"

if [ $? -eq 0 ]; then
    echo ""
    echo "🎉 Python dependencies installation complete!"
    echo "📁 Now run: colcon build (or ros2_foxy/cross_compile_foxy.sh) to install the Python node"
    echo "🚀 Launch with: ros2 launch drive_yolo motpy_tracking.launch.py"
else
    echo ""
    echo "💥 Installation had issues. Check the error messages above."
    exit 1
fi