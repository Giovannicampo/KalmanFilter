import numpy as np
import matplotlib.pyplot as plt

# Dati originali
x_data = np.array([0, 75, 150, 300, 450])
y_data = np.array([0, 14.28, 21.31, 25.68, 30.99])

# Calcolo dei coefficienti
n = len(x_data)
Σx = np.sum(x_data)
Σy = np.sum(y_data)
Σxy = np.sum(x_data * y_data)
Σx2 = np.sum(x_data**2)

m = (n * Σxy - Σx * Σy) / (n * Σx2 - Σx**2)
b = (Σy - m * Σx) / n

# Generazione dei punti sulla retta approssimata
x_line = np.linspace(0, 450, 100)
y_line = m * x_line + b

# Plot dei dati originali e della retta approssimata
plt.scatter(x_data, y_data, color='blue', label='Dati originali')
plt.plot(x_line, y_line, color='red', label='Retta approssimata')

plt.xlabel('Distanza (cm)')
plt.ylabel('Std_Dev')
plt.title('Appross. lineare (m=0.07)')
plt.legend()
plt.grid(True)
plt.show()

