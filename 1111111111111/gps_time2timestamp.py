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

gps_sec = []
gps_week = []
utc_list = []
timestamp_list = []
with open('span1.txt') as f:
    lines = f.readlines()[15:]
    for line in lines:
        gps_sec.append(line.split()[0])
        gps_week.append(line.split()[1])

gps_sec_float = map(float,gps_sec)
gps_week_float = map(float,gps_week)

for (i1,i2) in zip(gps_week_float,gps_sec_float):      
    utc = gps2utc(i1,i2)
    timestamp = utc2unix(utc)
    utc_list.append(utc)
    timestamp_list.append(timestamp)
    
for i,j in zip(utc_list,timestamp_list):
    print(i,j)
