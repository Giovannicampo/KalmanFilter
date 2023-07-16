import numpy as np
import matplotlib.pyplot as plt

# Dati originali
x_data = np.array([661, 750, 1500, 3000, 4500])
y_data = np.array([0.18, 0.46, 0.69, 0.83, 1])

# Calcolo dei coefficienti
n = len(x_data)
Σx = np.sum(x_data)
print(Σx)
Σy = np.sum(y_data)
print(Σy)
Σxy = np.sum(x_data * y_data)
print(Σxy)
Σx2 = np.sum(x_data**2)
print(Σx2)

m = (n * Σxy - Σx * Σy) / (n * Σx2 - Σx**2)
b = (Σy - m * Σx) / n
print('Coefficiente angolare: ', m)
print('Intercetta: ', b)

# Generazione dei punti sulla retta approssimata
x_line = np.linspace(0, 5000, 100)
y_line = m * x_line + b

# Plot dei dati originali e della retta approssimata
plt.scatter(x_data, y_data, color='blue', label='Dati originali')
plt.plot(x_line, y_line, color='red', label='Retta approssimata')

plt.xlabel('Distanza(mm)')
plt.ylabel('Std_Dev')
plt.title('Approssimazione lineare')
plt.legend()
plt.grid(True)
plt.show()

