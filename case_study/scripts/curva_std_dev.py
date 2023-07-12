import matplotlib.pyplot as plt

x = [0, 75, 150, 300, 450]
y = [0, 14.28, 21.31, 25.68, 30.99]

plt.plot(x, y, marker='o')
plt.xlabel('Distanza (cm)')
plt.ylabel('Std_Dev')
plt.title('Grafico della curva')
plt.grid(True)
plt.show()
