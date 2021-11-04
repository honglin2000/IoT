from tb_device_mqtt import TBDeviceMqttClient, TBPublishInfo
import serial
import time

thingsboard_server = 'thingsboard.cloud'
access_token = '350sGET8k4P4IMtV0BBA'


def main():	

	def on_server_side_rpc_request(client,request_id,request_body):
		if request_body['method'] == 'setLockValueB':
			global lock
			lock = bool(request_body['params'])
			if (lock):
				print('Entrance Lock')
			elif (lock == False):
				print('Entrance Lock')
		elif request_body['method'] == 'setTempValueB':
			global threshold
			threshold = float(request_body['params'])
			print(threshold)
			
	client = TBDeviceMqttClient(thingsboard_server, access_token)
	client.set_server_side_rpc_request_handler(on_server_side_rpc_request)
	client.connect()





f __name__ == "__main__":
    main()
    
    # Create TCP/IP socket
    print("Intializing TCP server... ", end="")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("", 12345))
    server_socket.listen(1)
    print("Initialized.")
