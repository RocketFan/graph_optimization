#!/bin/bash

topics=( $(rostopic list) )
uav_names=()
kill_pids=()

end() {
    for pid in "${kill_pids[@]}"
    do
        echo "Kill ${pid}"
        kill -9 $pid
    done
}

trap 'end' SIGINT

for topic in "${topics[@]}"
do
    topic_splitted=(${topic//// })
    if [[ "$topic_splitted" == *"uav"* ]]; then
        uav_names+=(${topic_splitted[0]})
    fi
done

IFS=$'\n'
uav_names_uniq=($(sort -u <<< ${uav_names[*]}))

for uav_name in "${uav_names_uniq[@]}"
do
    echo "Run ${uav_name}!"
    rosrun uwb_localization drone_controll_node.py _uav_name:=$uav_name &
    kill_pids+=($!)
done

sleep 999