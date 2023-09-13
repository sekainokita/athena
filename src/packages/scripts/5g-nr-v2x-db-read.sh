#!/bin/bash

number=0

while :
do
  if [ $number -gt 2 ]; then
    break
  fi

  cat /tmp/db_v2x_rx_temp_writing.csv | tail -1
done
