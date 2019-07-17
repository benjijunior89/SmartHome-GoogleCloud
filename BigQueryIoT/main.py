from google.cloud import bigquery
import paho.mqtt.client as mqtt
import datetime
import os

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "./IoT-UMA-a22c32a8dd3a.json"

clientCloud = bigquery.Client()
dataset_id = 'midataset' # replace with your dataset ID
table_id = 'proyecto_final' # replace with your table ID
table_ref = clientCloud.dataset(dataset_id).table(table_id)
table = clientCloud.get_table(table_ref) # API request

auth = {'username': "try", 'password':"try"}

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("casa/habitacion/cloud/temp")
    client.subscribe("casa/habitacion/cloud/luz")

def on_message(client, userdata, msg):
    #print(f"'@{str(msg.topic)} || {msg.payload}")
    today = datetime.date.today()
    d = {'Year': today.year, 'Month': today.month, 'Day': today.day}
    if(msg.topic == "casa/habitacion/cloud/temp"):
        print("RECIEVED - LUZ")
        d['Origen'] = "Temperatura"
        d['Valor'] = float(msg.payload)
        print(d)
        rows_to_insert = [d]
        errors = clientCloud.insert_rows(table, rows_to_insert)
        print(errors)
    elif(msg.topic == "casa/habitacion/cloud/luz"):
        print("RECIEVED - LUZ")
        d['Origen'] = "Luz"
        d['Valor'] = float(msg.payload)*100/1024
        print(d)
        rows_to_insert = [d]
        errors = clientCloud.insert_rows(table, rows_to_insert)
        print(errors)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("try","try")
client.connect("broker.shiftr.io", 1883, 60)

client.loop_forever()
### FALTA HACER QUE ESCUCHE A 