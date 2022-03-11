#!/usr/bin/python3
# coding=utf8
import sys

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

def reset():
    return None

def exit():
    print("RemoteControl Exit")
    return None

def start():
    print("RemoteControl Start")
    return None

def init():
    print("RemoteControl Init")
    return None

def stop():
    print("RemoteControl Stop")
    return None

def run(img):
    return img
