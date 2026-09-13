#!/bin/bash
echo "Iniciando micro_ros_agent..."
while true; do
    if [ -e /tmp/wokwi-tty ]; then
        ros2 run micro_ros_agent micro_ros_agent serial --dev /tmp/wokwi-tty -b 115200
        echo "O agente caiu. Tentando novamente em 2 segundos..."
    else
        echo "Aguardando ponte Wokwi (/tmp/wokwi-tty) ser criada..."
    fi
    sleep 2
done
