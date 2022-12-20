file_wp = open("wp.txt","r") # read mode
line = file_wp.readlines()
print(line)
print(len(line))

a = line[0]
b = a.split()
print((b[0]))
print((b[1]))

file_wp.close()