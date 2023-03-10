from time import time

snail = 0
sun = 5
night = 3
distance = 1000000000000
day = 0

start_time = time()

# while True:
#         day += 1
#         snail += 5
#         if snail >= distance:
#                 break
#         else:
#                 snail -= 3

day = ((distance - night) // (sun - night)) + 1

end_time = time()

print("time:", end_time - start_time)
print("day:", day)