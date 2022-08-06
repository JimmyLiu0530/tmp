#!/bin/bash
declare -a ap_formation=("average-channel-gain" "channel-gain" "strongest-ue" "weakest")
declare -a off=("exhaustive" "more_exhaustive")
declare -a backhaul=(1e100 1e9 8e8 6e8 4e8)
declare -a ue_dist=(1)
declare -a interf_index=(0.5 1.0 1.5 2.0)
declare -a ue_num=(20)

for i in {1..220};
    do
    for u in "${ue_num[@]}";
        do
        for b in "${backhaul[@]}";
                do
		for du in "${ue_dist[@]}";
			do
                        for method in "${ap_formation[@]}";
                                do
                                for of in "${off[@]}";
                                        do
                                        echo $method
                                        ./waf --run thesis << EOF
$u
$b
65
$du
200
dynamic
random
$method
$of
1
average
approximate
EOF
                done
        done
done
done
done
done
