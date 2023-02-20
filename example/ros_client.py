import roslibpy

client = roslibpy.Ros(host='127.0.0.1', port=9090)
client.run()

