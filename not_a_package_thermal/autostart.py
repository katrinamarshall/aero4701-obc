#! /usr/bin/env python
#from subprocess import call
#call([‘espeak “Welcome to the world of Robots” 2>/dev/null’], shell=True)
import datetime
import time

def log_timestamps_to_file(duration_seconds, filename):
    start_time = datetime.datetime.now()
    end_time = start_time + datetime.timedelta(seconds=duration_seconds)
    
    with open(filename, 'w') as file:
        while datetime.datetime.now() <= end_time:
            current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(current_time)  # Optional: Print to console
            file.write(current_time + '\n')
            time.sleep(1)  # Delay for 1 second

log_timestamps_to_file(10, 'timestamps.txt')
