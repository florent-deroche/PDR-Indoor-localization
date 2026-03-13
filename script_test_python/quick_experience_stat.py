import paho.mqtt.client as mqtt 
import json
import math
import statistics
import time
from scipy.optimize import least_squares

# --- CONFIGURATION ---
IP_ADRESS   = '10.120.2.235'
PORT        = 1883
TOPIC       = "PDR/robot/rssi"
DUREE_TEST  = 30 # Secondes d'enregistrement

ANCHORS_POS = {
    'Anchor_1': (2.15, 0.0),
    'Anchor_2': (1.05, 1.3),
    'Anchor_3': (0.0, 0.0)
}

POSITION_REELLE = (1.05, 0.68)
A_CONST = -33.0 
N_CONST = 3.8   

# --- LISTES POUR LES STATISTIQUES ---
estimations_x = []
estimations_y = []
erreurs_distance = []
temps_debut = None

# --- CLASSES ET FONCTIONS (Identiques à ton main.py) ---
class KalmanFilter1D:
    def __init__(self, q, r, p, initial_value):
        self.q = q
        self.r = r
        self.p = p
        self.x = initial_value

    def process(self, measurement, is_valid):
        self.p = self.p + self.q
        if is_valid:
            k = self.p / (self.p + self.r)
            self.x = self.x + k * (measurement - self.x)
            self.p = (1 - k) * self.p
        return self.x

filtres_ancres = {
    'Anchor_1': KalmanFilter1D(q=0.05, r=25.0, p=1.0, initial_value=-60.0),
    'Anchor_2': KalmanFilter1D(q=0.05, r=25.0, p=1.0, initial_value=-60.0),
    'Anchor_3': KalmanFilter1D(q=0.05, r=25.0, p=1.0, initial_value=-60.0)
}

def rssi_to_distance(rssi):
    return 10 ** ((A_CONST - rssi) / (10 * N_CONST))

def error_function(guess, anchors, distances):
    errors = []
    for (ax, ay), d in zip(anchors, distances):
        calc_dist = math.sqrt((guess[0] - ax)**2 + (guess[1] - ay)**2)
        errors.append(calc_dist - d)
    return errors

def calculate_position(distances_dict):
    anchors_list = []
    distances_list = []
    
    for ancre, dist in distances_dict.items():
        anchors_list.append(ANCHORS_POS[ancre])
        distances_list.append(dist)
    
    initial_guess = [1.0, 0.5] 
    result = least_squares(error_function, initial_guess, args=(anchors_list, distances_list))
    
    # --- AJOUT DE LA RECALIBRATION (OFFSET) ---
    x_brut = result.x[0]
    y_brut = result.x[1]
    
    # On corrige le biais :
    # Si le biais X est de -0.375, on fait : x_brut - (-0.375) = x_brut + 0.375
    # Si le biais Y est de +0.732, on fait : y_brut - (+0.732) = y_brut - 0.732
    x_corrige = x_brut - 0.882
    y_corrige = y_brut - 0.124
    
    return round(x_corrige, 2), round(y_corrige, 2)
# --- CALLBACKS MQTT ---
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print(f"Connecté. Lancement du diagnostic de {DUREE_TEST} secondes...")
        global temps_debut
        temps_debut = time.time()
        client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    try:
        donnees = json.loads(msg.payload.decode('utf-8'))
        distances = {}
        
        for ancre, rssi in donnees.items():
            mesure_valide = (rssi != -100)
            rssi_filtre = filtres_ancres[ancre].process(measurement=rssi, is_valid=mesure_valide)
            distances[ancre] = rssi_to_distance(rssi_filtre)

        x, y = calculate_position(distances)
        
        # Enregistrement pour les statistiques
        estimations_x.append(x)
        estimations_y.append(y)
        
        erreur = math.sqrt((x - POSITION_REELLE[0])**2 + (y - POSITION_REELLE[1])**2)
        erreurs_distance.append(erreur)
        
    except Exception as e:
        pass # On ignore les erreurs de format pendant le diagnostic

# --- LANCEMENT ET CALCUL DES STATS ---
if __name__ == '__main__':
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    
    client.connect(IP_ADRESS, PORT, 60)
    client.loop_start()
    
    # Boucle d'attente
    while temps_debut is None or (time.time() - temps_debut) < DUREE_TEST:
        time.sleep(0.1)
        
    client.loop_stop()
    client.disconnect()
    
    # --- CALCUL DES STATISTIQUES ---
    N = len(estimations_x)
    print(f"\n========== RAPPORT DE DIAGNOSTIC ({N} mesures) ==========")
    
    if N > 2:
        # 1. Moyennes
        moyenne_x = statistics.mean(estimations_x)
        moyenne_y = statistics.mean(estimations_y)
        
        # 2. Biais (Erreur systématique)
        biais_x = moyenne_x - POSITION_REELLE[0]
        biais_y = moyenne_y - POSITION_REELLE[1]
        
        # 3. Écart-type (Bruit / Tremblement)
        std_x = statistics.stdev(estimations_x)
        std_y = statistics.stdev(estimations_y)
        
        # 4. Erreurs absolues
        mae = statistics.mean(erreurs_distance) # Mean Absolute Error
        rmse = math.sqrt(sum(e**2 for e in erreurs_distance) / N) # Root Mean Square Error
        
        print(f"POSITION MOYENNE ESTIMÉE : ({moyenne_x:.2f}, {moyenne_y:.2f})")
        print(f"Position réelle attendue : {POSITION_REELLE}")
        print(f"Biais en X : {biais_x * 100:+.1f} cm ")
        print(f"Biais en Y : {biais_y * 100:+.1f} cm ")
        print(f"Écart-type X : ± {std_x * 100:.1f} cm")
        print(f"Écart-type Y : ± {std_y * 100:.1f} cm")
        print(f"Erreur moyenne (MAE) : {mae * 100:.1f} cm")
        print(f"Score RMSE           : {rmse * 100:.1f} cm")
    else:
        print("Pas assez de données reçues pour faire des statistiques.")