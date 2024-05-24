#!/bin/sh

LOG=temperature
LOGDIR=$(pwd)/$LOG
LOGFILE=$(date +"%y_%m_%d"_$LOG)

TEMPDIR=/sys/class/thermal/thermal_zone0/temp
NATURALTEMP=$(cat $TEMPDIR)
TEMP=$(expr $NATURALTEMP / 1000)

NOW=$(date +"%T")
if [ ! -d $LOGDIR ];then
  mkdir $LOGDIR
fi

echo [ $NOW ] Start! >> $LOGDIR/$LOGFILE

for (( ; ; ))
do
  NOW=$(date +"%T")
  NATURALTEMP=$(cat $TEMPDIR)
  TEMP=$(expr $NATURALTEMP / 1000)
  echo [ $NOW ] $TEMP C >> $LOGDIR/$LOGFILE
  echo [ $NOW ] $TEMP C

  sleep 10
done
