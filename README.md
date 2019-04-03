# gpstime2unixtime
`span1.txt` is an example file which contains positioning results from SPAN-CPT.
`gps_time2timestamp.py` reads the first column of the txt file which is gps time seconds, 
notice that we skip numbers of rows since they are some head statements. 
Then we use two functions to transform gps time to UTC time to unix unix time (based on +8 time zone China).
And store the transformed unix time in a list for further use.
`gps2timestamp_copy.py` is general version doing the transformation.

This file is just for personal use, if you have any questions please feel free to contact me.
Auother: Hu Sai
Email: 17095338g@connect.polyu.hk
