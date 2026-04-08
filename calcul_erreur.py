import pandas as pd
import numpy as np

file_name = 'positions.csv'
df = pd.read_csv(file_name)

# Erreur Euclidienne instantanée pour chaque algorithme
df['erreur_ble'] = np.sqrt((df['x_ble'] - df['x_slam'])**2 + (df['y_ble'] - df['y_slam'])**2)
df['erreur_kalman'] = np.sqrt((df['x_kalman'] - df['x_slam'])**2 + (df['y_kalman'] - df['y_slam'])**2)

df['diff_ble'] = np.sqrt(df['x_ble'].diff()**2 + df['y_ble'].diff()**2)
df['diff_kalman'] = np.sqrt(df['x_kalman'].diff()**2 + df['y_kalman'].diff()**2)

# MSE
mae_ble = df['erreur_ble'].mean()
mae_kalman = df['erreur_kalman'].mean()

# RMSE
rmse_ble = np.sqrt((df['erreur_ble']**2).mean())
rmse_kalman = np.sqrt((df['erreur_kalman']**2).mean())

# Jitter moyen
jitter_ble = df['diff_ble'].mean()
jitter_kalman = df['diff_kalman'].mean()

print(f"MAE BLE : {mae_ble:.3f}m")
print(f"MAE Kalman : {mae_kalman:.3f}m")
print(f"RMSE BLE : {rmse_ble:.3f}m")
print(f"RMSE Kalman : {rmse_kalman:.3f}m")
print(f"Jitter moyen BLE : {jitter_ble:.3f}m")
print(f"Jitter moyen Kalman : {jitter_kalman:.3f}m")
