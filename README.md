# Energiby Refa
Bue &amp; Co. - Energiby Refa

# What is needed
1. Raspberry Pi 4
2. Teensy 4.1

# Notes - Install PI
Download Raspberry Pi Imager https://www.raspberrypi.com/software/
  - Write Raspberry PI OS (32-BIT) to a 16GB SD Card


Install Needed Packages!
```console
sudo apt install -y code
sudo apt install -y python3 python3-pip python3-numpy python3-matplotlib
sudo pip3 install python-osc
```

# Install Arduino
Download Arduino IDE: https://downloads.arduino.cc/arduino-1.8.19-linuxarm.tar.xz

Install Arduino IDE
Open a terminal window and Navigate to the Downloads folder:
```console
cd ~/Downloads
```

List the files in the Downloads folder using:
```console
ls
```

You should see the Arduino IDE archive:
```console
arduino-####-linuxarm.tar.xz
```
Note the version number.

Extract the contents of the downloaded file:
```console
tar -xf arduino-####-linuxarm.tar.xz
```
This should create a folder named “arduino-####” full of files.

Move the folder to /opt using:
```console
sudo mv arduino-#### /opt
```
Finally complete the installation by running:
```console
sudo /opt/arduino-####/install.sh
```

# Install Teensyduino
Download Teensyduino: https://www.pjrc.com/teensy/td_156/TeensyduinoInstall.linuxarm

Linux Installation
Download the Linux udev rules https://www.pjrc.com/teensy/00-teensy.rules and copy the file to /etc/udev/rules.d.
```console
sudo cp 00-teensy.rules /etc/udev/rules.d/
```
Run the installer by adding execute permission and then execute it.
```console
chmod 755 TeensyduinoInstall.linux64
./TeensyduinoInstall.linux64
```
Install missing package
```console
sudo apt-get install libusb-0.1-4
```

# Start Application on Boot
This is acomplished by creating a systemd service with filename ```energiby.service```:
```bash
[Unit]
Description=Start Energiby

[Service]
Environment=DISPLAY=:0
Environment=XAUTHORITY=/home/pi/.Xauthority
ExecStart=/usr/bin/python3 /home/pi/Code/energiby/energiby.py
Restart=always
RestartSec=10s
KillMode=process
TimeoutSec=infinity

[Install]
WantedBy=graphical.target
```

# Hide Mouse Pointer on Boot
```bash
sudo sed -i -- "s/#xserver-command=X/xserver-command=X -nocursor/" /etc/lightdm/lightdm.conf
```

# Access
--- RaspberryPi ---
```
User : pi
Password : EnergibyRefa
```
--- Router ---
```
User : admin
Password : Energiby3
```
--- WLAN ---
```
SSID : EnergibyRefa
Password : Energiby3
```
