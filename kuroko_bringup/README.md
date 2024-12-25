kuroko_bringup
====
  
### Setup
copy udev rules
```
sudo cp udev/rules.d/99-kuroko-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
