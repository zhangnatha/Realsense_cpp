#!/bin/bash

echo "正在检查Intel RealSense设备..."

# 检查是否有RealSense设备
if lsusb | grep -q "Intel"; then
    echo "找到Intel设备:"
    lsusb | grep Intel
else
    echo "未找到Intel设备"
    exit 1
fi

echo ""
echo "正在尝试重置USB设备..."

# 查找RealSense设备的USB ID
USB_ID=$(lsusb | grep Intel | head -1 | awk '{print $6}')

if [ -n "$USB_ID" ]; then
    echo "找到设备ID: $USB_ID"
    
    # 尝试重置设备
    echo "正在重置设备..."
    sudo usb-reset "$USB_ID" 2>/dev/null || {
        echo "usb-reset命令不可用，尝试其他方法..."
        
        # 尝试重新加载USB驱动
        echo "重新加载USB驱动..."
        sudo modprobe -r uvcvideo 2>/dev/null
        sleep 2
        sudo modprobe uvcvideo 2>/dev/null
        
        echo "等待设备重新初始化..."
        sleep 5
    }
    
    echo "USB设备重置完成"
else
    echo "无法获取设备ID"
    exit 1
fi

echo ""
echo "检查设备状态..."
lsusb | grep Intel

echo ""
echo "如果问题仍然存在，请尝试："
echo "1. 拔插USB线缆"
echo "2. 重启计算机"
echo "3. 检查是否有其他程序正在使用相机" 