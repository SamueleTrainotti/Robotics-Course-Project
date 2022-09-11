rm -rf imgs/*
rosrun image_view image_saver image:=/camera/Scene/image_raw "_filename_format:=imgs/left%04i.%s" &
pidd=$!
echo "$pidd"
sleep 4
kill -s STOP $pidd
echo "killed"
