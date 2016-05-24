for j in {0..4} # model type
do
    echo "$j"
    for k in {0..49} #49 # trial
    do
	echo "$j$k"
	./testRealWorld.o $j 10 1 3 0
    done
done