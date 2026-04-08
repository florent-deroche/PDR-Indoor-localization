import pandas as pd
import numpy as np

file_name = 'positions.csv'
df = pd.read_csv(file_name)

# Erreur Euclidienne instantanée pour chaque algorithme
df['erreur_ble'] = np.sqrt((df['x_ble'] - df['x_slam'])**2 + (df['y_ble'] - df['y_slam'])**2)
df['erreur_kalman'] = np.sqrt((df['x_kalman'] - df['x_slam'])**2 + (df['y_kalman'] - df['y_slam'])**2)

# RMSE
rmse_ble = np.sqrt((df['erreur_ble']**2).mean())
rmse_kalman = np.sqrt((df['erreur_kalman']**2).mean())

print(f"RMSE BLE : {rmse_ble:.3f} mètres")
print(f"RMSE Kalman : {rmse_kalman:.3f} mètres")