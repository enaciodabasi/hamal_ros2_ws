#!/bin/bash

launch_file_name="hamal_startup.launch.py"

launch_process_id=$(ps -ef | awk '$NF=='"${launch_file_name}"'')
kill $launch_process_id
while kill -0 $launch_process_id 2>/dev/null; do sleep 0.2; done