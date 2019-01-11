# Initial setup

## Write Raspbian image to SD card
### Windows
TODO

### Mac
```
$ diskutil list
/dev/disk0 (internal):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      GUID_partition_scheme                         1.0 TB     disk0
   1:                        EFI EFI                     314.6 MB   disk0s1
   2:                 Apple_APFS Container disk1         999.6 GB   disk0s2

/dev/disk1 (synthesized):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      APFS Container Scheme -                      +999.6 GB   disk1
                                 Physical Store disk0s2
   1:                APFS Volume Macintosh HD            158.0 GB   disk1s1
   2:                APFS Volume Preboot                 22.2 MB    disk1s2
   3:                APFS Volume Recovery                509.8 MB   disk1s3
   4:                APFS Volume VM                      4.3 GB     disk1s4

/dev/disk2 (external, physical):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:     FDisk_partition_scheme                        *32.0 GB    disk2
   1:             Windows_FAT_32 NO NAME                 32.0 GB    disk2s1

$ diskutil unmountDisk /dev/disk2
Unmount of all volumes on disk2 was successful
$ sudo dd if=2017-11-29-raspbian-stretch.img of=/dev/rdisk2 bs=5m
Wait until it is done, Ctrl+T will show progress
$ diskutil eject /dev/disk2
```
## Install software
### update OS (internet access required)
```
sudo apt-get update
sudo apt-get upgrade
```
## enable SSH and VNC
```
$ sudo raspi-config
```
Interfacing Options -> Enable SSH and VNC

## Install Windows Remote Desktop Access
```
sudo apt-get install xrdp
```

## Access Raspberry PI from Mac
```
ssh pi@raspberrypi.local
password: raspberry
```

## Enable PI camera
Note that this is not for usb camera, please refer to next section
```
sudo raspi-config
reboot
raspistill -vf -hf -o test2.jpg
```
## Enable USB camera
```
sudo apt-get install fswebcam
fswebcam -r 1920x1080 test5.jpg
```

## Face detection
Installs opencv for python, matplotlib (a graphing utility)
```
sudo apt-get install python-opencv
sudo apt-get install python-matplotlib
```

Downloads images for facedetection and "Mona Lisa" painting for testing
```
wget https://raw.githubusercontent.com/shantnu/Webcam-Face-Detect/master/haarcascade_frontalface_default.xml

wget -O face.jpg https://upload.wikimedia.org/wikipedia/commons/thumb/e/ec/Mona_Lisa%2C_by_Leonardo_da_Vinci%2C_from_C2RMF_retouched.jpg/687px-Mona_Lisa%2C_by_Leonardo_da_Vinci%2C_from_C2RMF_retouched.jpg
```

### Example face.py
Simple face detection script which looks at face.png file
```python
import io
import picamera
import cv2
import numpy

image = cv2.imread('face.jpg')

#Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

#Convert to grayscale
gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

#Look for faces in the image using the loaded cascade file
faces = face_cascade.detectMultiScale(gray, 1.1, 5)

print "Found "+str(len(faces))+" face(s)"

#Draw a rectangle around every found face
for (x,y,w,h) in faces:
    cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)

#Save the result image
cv2.imwrite('result.jpg',image)
```
### Run with
```
python face.py
```

## Install package for communication with RoboRio
```
$ pip install pynetworktables
```
