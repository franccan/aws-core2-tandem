#!/bin/bash 

#socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\" & 

#export DISPLAY=192.168.1.14:0
export IMAGE_NAME=aws-core2-tandem-build

 docker run -it -v $(pwd):/local  --privileged -v /dev:/dev $IMAGE_NAME bash