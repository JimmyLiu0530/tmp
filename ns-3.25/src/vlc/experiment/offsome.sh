#!/bin/bash
declare -a ap_formation=("average-channel-gain" "channel-gain" "strongest-ue")
declare -a off=("OffSome")
declare -a backhaul=(1e9 1e100)
declare -a ue_dist=(3 5 7 9)
declare -a interf_index=(0.5 1.0 1.5 2.0)
declare -a ue_num=(20)


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
					for val in "${interf_index[@]}";
						do
						for i in {1..220};
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
$val
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
done
