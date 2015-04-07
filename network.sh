#!/bin/bash

echo "::Network Information::\n"
netstat -rn && ifconfig

echo "::Network Validation::\n"
echo "Local Network:"
ping -c3 192.198.1.99

sleep 1

echo "\nInternet Access:"
ping -c3 8.8.8.8
