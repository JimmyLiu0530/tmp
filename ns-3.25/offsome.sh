#!/bin/bash
declare -a ap_formation=("channel-gain" "strongest-ue")
declare -a off=("OffSome")
declare -a ue_dist=(5 7 1)
declare -a interf_index=(2.0)
declare -a ue_num=(20)
declare -a backhaul=(1e9)

for u in "${ue_num[@]}";
     do
     for b in "${backhaul[@]}";
                do
		for method in "${ap_formation[@]}";
			do
                        for of in "${off[@]}";
			        do
				for val in "${interf_index[@]}";
					do
					for du in "${ue_dist[@]}";
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
