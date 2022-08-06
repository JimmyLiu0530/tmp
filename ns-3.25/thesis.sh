#!/bin/bash
declare -a off=("exhaustive" "more_exhaustive")
declare -a backhaul=(1e9)
declare -a ue_dist=(12 13 14 15 16)
declare -a interf_index=(0.5 1.0 1.5 2.0)
declare -a ue_num=(20)
declare -a ap_formation=("average-channel-gain")

    for u in "${ue_num[@]}";
        do
        for b in "${backhaul[@]}";
                do
		for du in "${ue_dist[@]}";
			do
                        for method in "${ap_formation[@]}";
                                do
					for i in {1..100};
					do
                                        	echo $method
                                        	./waf --run thesis << EOF
$u
$b
65
$du
200
k-means
random
$method
OffSome
2
1
average
approximate
EOF
                done
        done
done
done
done
