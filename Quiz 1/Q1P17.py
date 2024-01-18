a=[1,2,3,4,5,0]
b=[0,0,0,0,0]

for i in range(len(b)):
    b[i] = a[i] + a[i + 1]

# The original error was caused by "a[i + 1]", where i was increased to the maximum
# index of a. There are a few ways to solve this, but adding a 0 to the variable a and
# changing the range to the length of b achieves the desired output.