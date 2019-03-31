import random
n =  int(input("Masukan n : "))
f = open("maze.txt", "w+")
for i in range(n):
    for j in range(n):
        r = random.randint(1,5)
        if (r < 5  and (i > 0 and i < n-1 and j>0 and j<n-1)) or (i == 1 and j == 0) or (i == n-2 and j ==n-1):
            f.write('0')
        else:
            f.write('1')
    f.write("\n")