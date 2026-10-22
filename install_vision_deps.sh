#!/bin/bash
# 安装视觉处理所需的 Python 依赖

set -e

echo "=========================================="
echo "  安装视觉处理依赖"
echo "=========================================="
echo ""

echo "安装 Python OpenCV 和 NumPy..."
sudo apt-get install -y python3-opencv python3-numpy

echo ""
echo "验证安装..."
python3 -c "import cv2; print('OpenCV 版本:', cv2.__version__)"
python3 -c "import numpy; print('NumPy 版本:', numpy.__version__)"

echo ""
echo "=========================================="
echo "  安装完成！"
echo "=========================================="
