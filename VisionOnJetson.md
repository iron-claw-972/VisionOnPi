# Initial setup

## Write Jetpack image to SD card

It is strongly recommended to use a 64 GB or larger SD card.

https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write

# First login

## Connect peripherals

Connect your monitor, keyboard, mouse and power supply to the Jetson Nano. The Jetson Nano should boot after a few seconds.

## First boot setup

The most important things to do are to set the username / "your name" to `pi`, the password to `raspberry` and the device name to `jetson`. This is in an attempt to stop people from losing the password for a Jetson. After this point, you can skip through most of the other config.

# Important installs

## Use of sudo

`sudo` is a keyword that can be added to almost any command in the terminal. Adding `sudo`, means that the next command will be run with administrator privileges. When run, the terminal will ask for the admin password, which should be `raspberry`.

## Networking

In order to complete the next steps, one must connect the Jetson Nano to the internet. This can either be achieved through an ethernet connection to the Jetson or through Wifi. In order to connect through wifi, one needs a USB wifi card.

## Swap file

In order to set up a swap file, which allows for the system to use some of the SD storage as additional RAM, one should run the command:

```
git clone https://github.com/JetsonHacksNano/installSwapfile
```

Then switch the directory to inside the downloaded folder:

```
cd installSwapfile
```

After which, the command:

```
./installSwapfile
```

Further directions can be found [here.](https://www.jetsonhacks.com/2019/04/14/jetson-nano-use-more-memory/)

## Download pip

pip is a very useful tool for downloading libraries for python.

Run:

```
cd ~
```

in order to navigate back to the home directory. Then run:

```
wget https://bootstrap.pypa.io/get-pip.py
```

to download,

```
sudo python3 get-pip.py
```

to run and

```
rm get-pip.py
```

to remove the script that downloaded pip.

## jtop

`jtop` is a useful tool for seeing the utilization of the jetson's CPU and GPU. It also provides an array of other useful information about the jetson. In order to download, run:

```
sudo -H pip install jetson-stats
```

In order to use `jtop`, one should run the command `sudo jtop`, which will open a ascii GUI in the terminal. In order to navigate, the keyboard should be used.

## OpenCV with CUDA

In order to use the GPU of the jetson for OpenCV based vision processing, they must compile OpenCV for CUDA. CUDA is Nvidia's architecture for interfacing with their GPUs. This is achieved in a few simple steps. First, you should navigate to the home directory using `cd ~`. Next clone the repo like so:

```
git clone https://github.com/JetsonHacksNano/buildOpenCV
```

Then navigate to inside the repo, using:

```
cd buildOpenCV
```

Finally, start the build of OpenCV for CUDA. *Please note that this will take multiple hours to complete. It is also critical to have setup the swap file by this point, as without it, the build will fail.*

```
./buildOpenCV.sh |& tee openCV_build.log
```

Now just wait...

## pynetworktables

In order to install networktables, use the command:

```
pip3 install pynetworktables
```
