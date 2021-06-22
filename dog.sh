#!/bin/bash
echo ubuntu|sudo -S sudo chmod +777 /dev/ttyUSB0
echo ubuntu|sudo -S sudo chmod +777 /dev/ttyUSB1

sec=1
cnt=0
name=Domino
Thread=`ps -ef | grep $name | grep -v "grep"`
cd /home/domino/robocon/build/
make clean && make -j
while [ 1 ]
do
count=`ps -ef | grep $name | grep -v "grep" | wc -l`
echo "Thread count: $count"
echo "Expection count: $cnt"
if [ $count -gt 1 ]; then
    echo "The $name is still alive!"
    sleep $sec
else 
    echo "Starting $name..."
    
    cd /home/domino/robocon/build/
    bash -c "./$name;exec bash;"
    echo "$name has started!"		
    sleep $sec
    ((cnt=cnt+1))
    
fi
done