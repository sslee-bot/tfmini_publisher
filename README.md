# tfmini_publisher

## prerequisite

- install
```bash
sudo apt install python3-pip
pip3 install serial pyserial
```

- enable UART

```bash
# Add following line into /boot/firmware/config.txt
enable_uart=1

# Then
sudo chmod 777 /dev/ttyS0
```
