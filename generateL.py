import numpy as np

a = ["rrrGGgrrrGGg", "rrryygrrryyg", "rrrrrGrrrrrG", "rrrrryrrrrry", "GGgrrrGGgrrr", "yygrrryygrrr", "rrGrrrrrGrrr", "rryrrrrryrrr"]


numLanes = 4

numConnections = len(a[0])

b = []

for lane in range(0,numConnections):
    
    d = [0] * (numConnections)
    
    for item in a:
        
        c = list(item)
        
        for letter in range(0,numConnections):

            if (c[lane] == "G" or c[lane] == "g") and (c[letter] == "G" or c[letter] == "g") :
                d[letter] = 1
                
    b.append(d)
        
        
print(b)

e = np.matrix(b)

print(e)


strings = []
for set in b:
    new_string = []
    for light in set:
        if light > 0:
            new_string.append('r')
        else:
            new_string.append('G')
    strings.append("".join(new_string))
    
print(strings)