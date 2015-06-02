for j in 4 # model type
do
    echo "$j"
    for k in {0..3} # trial
    do
	echo "$j$k"
	./testRealWorld.o $j 10 1 3 0
    done
done