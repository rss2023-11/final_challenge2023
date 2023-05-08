# Setup Stopsign Detector
This file contains instructions to set up the stopsign detector in a new environment. This must be run before the stopsign detector can work.

## Step 1: Install python3.
In order for pytorch to work properly, we need at least Python 3.6. Here are instructions to download Python 3.9. Unfortunately,
we have to compile from source because Ubuntu 9 doesn't have a package for anything later than 3.5.

First, install the system package
```
sudo apt-get install libffi-dev
```

This is necessary for `ctypes` to work and must be installed before Python 3.9 is compiled.

Next, download the source code for Python 3.9 and extract it:
```sh
wget https://www.python.org/ftp/python/3.9.4/Python-3.9.4.tgz
tar xzf Python-3.9.4.tgz
```

Next, `cd` to the extracted directory.
```sh
cd Python-3.9.4
```

Finally, enable compiling optimizations and compile it:
```sh
./configure --enable-optimizations
sudo make altinstall
```
(Note: `altinstall` installs python3.9 alongside other installations of Python instead of replacing them. This is to make sure we don't break any system packages
that rely on particular versions of Python 3.)

## Step 2: Install various system packages
```
sudo apt install python3-tk
```

## Step 3: Install required python3 packages
Go to the `city_driving` directory and run `pip3.9 install -r python3_requirements.txt`.