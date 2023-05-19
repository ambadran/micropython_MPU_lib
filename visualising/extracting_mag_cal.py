import pyperclip

x = []
y = []
z = []

with open("log.txt", 'r') as f:
    for line in f.readlines():
        try:
            current = eval(line)
            x.append(current[0])
            y.append(current[1])
            z.append(current[2])
        except Exception:
            pass


pyperclip.copy(str(x))
k = input("copied x, press enter to copy y")
pyperclip.copy(str(y))
k = input("copied y, press enter to copy z")
pyperclip.copy(str(z))
print("copied z")

