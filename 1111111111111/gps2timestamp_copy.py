##!/usr/bin/python3
# -*- coding: utf-8 -*-
# @Time     : 2019/4/2
# @Author   : Hu Sai
# @Filename : gps_time2timestamp.py
# @Email    : 17095338g@connect.polyu.hk

import datetime,time,calendar
import numpy as np
        
def gps2utc(gpsweek,gpsseconds):
    datetimeformat = "%Y-%m-%d %H:%M:%S"
    epoch = datetime.datetime.strptime("1980-01-06 00:00:00",datetimeformat)
    elapsed = datetime.timedelta(days=(gpsweek*7),seconds=(gpsseconds))
    return datetime.datetime.strftime(epoch + elapsed,datetimeformat)

def utc2unix(utc):
    timeArray = time.strptime(utc, "%Y-%m-%d %H:%M:%S")
    timeStamp = int(time.mktime(timeArray)) + 28800
    return timeStamp

utc = gps2utc(2047,34548.05)
timestamp = utc2unix(utc)
print(utc)
print(timestamp)