import socket
import struct
import time
import sys

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9001))
    
    # Send a dummy packet to the bridge to subscribe
    print("Subscribing to balancer_simulator on UDP 9000...")
    sock.sendto(b'hello', ('127.0.0.1', 9000))
    
    # Listen for incoming messages
    sock.settimeout(5.0)
    print("Listening for relayed IPC messages...")
    received_any = False
    
    start_time = time.time()
    while time.time() - start_time < 3.0:
        try:
            data, addr = sock.recvfrom(4096)
            if len(data) >= 2:
                msg_id = struct.unpack('<H', data[:2])[0]
                print(f"Received msg (id={msg_id}) length={len(data)} from {addr}")
                received_any = True
            else:
                print(f"Received malformed {data} from {addr}")
        except socket.timeout:
            break

    if not received_any:
        print("FAIL: No messages received.")
        sys.exit(1)
    else:
        print("SUCCESS: Received messages from the bridge.")
        sys.exit(0)

if __name__ == '__main__':
    main()
