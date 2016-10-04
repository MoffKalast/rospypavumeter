#!/bin/sh

cd /tmp
git clone https://github.com/Valodim/python-pulseaudio

cd python-pulseaudio
sudo python setup.py install
