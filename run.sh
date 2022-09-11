rm -rf src/yolov5/runs/detect/*

python3 src/yolov5/detect.py --source imgs/left0000.jpg --weights src/yolov5/runs/train/exp22/weights/best.pt --conf 0.70 --save-txt --save-crop

