import paho.mqtt.client as mqtt
import json


IP_ADRESS   = '10.120.2.231'
PORT        = 1883
TOPIC       = "PDR/robot/rssi"


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connexion réussie")
        client.subscribe(TOPIC)
    else:
        print("Connexion échouée")

def on_message(client, userdata, msg):
    try:
        payload_texte = msg.payload.decode('utf-8')
        donnees = json.loads(payload_texte)
        print(f"Nouveau message reçu : {donnees}")
        
    except json.JSONDecodeError:
        print("Erreur : Le message reçu n'est pas un format JSON valide.")
        # Correction ici : on affiche le message brut, pas la variable 'donnees' qui a échoué
        print(f"Données brutes : {msg.payload}")
    except Exception as e:
        print(f"Une erreur inattendue est survenue : {e}")

if __name__ == '__main__':
    # On précise la version 2 de l'API pour supprimer le DeprecationWarning
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    
    print(f"Connexion à {IP_ADRESS}:{PORT}...")
    client.connect(IP_ADRESS, PORT, 60)
    
    # La fameuse boucle qui empêche le script de se terminer tout de suite
    try:
        print("En écoute... (Fais Ctrl+C pour arrêter)")
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nArrêt manuel du script.")
        client.disconnect()
