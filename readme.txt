This Python code is for app controlling the Orion SkyQuest XT12g telescope focuser
and zoom eyepiece. It's all work-in-progress/experimental at the moment.

The eyepiece used here is Baader Hyperion 8-24mm focal length zoom eyepiece.
The remote control is built using two https://www.hiwonder.com/products/lx-225
bus servo motors and https://www.hiwonder.com/products/hiwonder-ttl-usb-debugging-board.

The 3D printed parts STLs are in the "./stls" folder. The belt for the zoom
is a vacuum’s belt (https://www.amazon.com/gp/product/B0B5KCH9FQ). 
For attaching the bracket to the focuser base use M5 x 20mm. The zoom needs
four 6-32 x 1/2 and one 6-32 x 1 1/4 (just cut to length a longer one) w/ nuts.
The included with the servo motors screws are too short for connecting
the 3D printed zoom wheel and focuser handle cap to the plastic horn wheels
received with the servos. Four longer screws for each wheel will be needed
(look for little brass screws in the hardware store, 9.4mm long measured with
the cap, can be of a bit larger diameter than the originals).
Note: the focuser bracket holding screws can be loosened and the bracket shifted
      left to allow manual focuser control when required.

The cell phones are used for cameras. Everything is connected to a mini PC.
The phones (one is used as a finder, another one is attached to the eyepiece)
are connected to the USB3 ports, the focuser/zoom control board and the telescope
go-to system are connected to the remaining USB2 ports of the mini PC. The mini PC
is attached to the telescope mount. It has WiFi and the remote terminal access is
enabled there to connect and control all the equipment in the remote terminal
session. Everything is fed from a 15AH LFP battery also attached to the telescope
mount. Lx225 motors need a reducer https://www.amazon.com/dp/B07SGJSLDL.

The phones are controlled using scrcpy (https://github.com/Genymobile/scrcpy).
The Orion's go-to system is controlled using SynScanPro app
(can be found here: https://skywatcher.com/download/software/synscan-app/)
through "SynScan USB" dongle (this is just a USB-TTL converter, you can buy one or
DIY, see: https://www.atm-workshop.com/synscan-pc-connect.html).

Setting up the windows 10/11 PC for running the Python app:
- Install python 3.11.4 (new pip is available, python.exe -m pip install --upgrade pip).
- VC redistributable: https://aka.ms/vs/17/release/vc_redist.x64.exe.
- Install wxPython ("pip install attrdict3 requests" then "pip install wxPython").
- Install wxWidgets ("pip install wxWidgets").
- Install pyserial ("pip install pyserial").
- Use "Bus Servo Terminal" (https://www.dropbox.com/sh/16ivgj1p1a6teja/AACODb-AmLcKe9VYSgN0q4sqa)
  to make sure the motors and the controller board work and to assign IDs for the zoom
  and focuser motors.
- Start the orionrc app, go to the settings tab and update settings to match your setup,
  click "Save" and say "yes" to restart the app, the app should be working after that.
