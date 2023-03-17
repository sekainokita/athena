#!/usr/bin/env python3

from random import *

origin_num = randint(0, 1001)
num = origin_num

if num <= 250:
    num = 250
elif num >= 750:
    num = 750
else:
    pass

print(f"origin_num:{origin_num}")
print(f"num:{num}")
