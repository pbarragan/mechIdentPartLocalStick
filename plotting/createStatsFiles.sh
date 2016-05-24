DATE=$(basename $1)
NAME='statsFiles'$DATE'.py'
echo $NAME

FILE_LIST=$(ls -rt $1/*.txt | awk '{print "\x27"$NF"\x27,"}')

cat <<EOF >$NAME
# this is for the <INSERT DATE> experiments
# no parameter variation. <INSERT NUMBER OF TRIALS> trials for each model type
# are these simulations?: <YES or NO>
# this is with <INSERT ACTION SELECTION TYPE> action selection
# sometimes depending on action validity, the action sequence is not identical
# is there bias?: <YES or NO>
# is there stick?: <YES or NO>

files = [$FILE_LIST]
EOF