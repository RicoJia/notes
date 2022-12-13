# 1. Instead of reloading the whole page after your browser requests to update a certain area
# The server can just send that piece back
# 2. Async requests are requests that don't block the client. Client can come back to process
# When the response is received.
# We can set up the http.server, which loads all files in its current directory. 
from urllib.request import urlopen
def test_http_server():
    u = urlopen("http://localhost:8000/scratch.md")
    u.read().decode("utf-8")

def test_socket_file_descriptor_passing():
    import multiprocessing 
    from multiprocessing.reduction import recv_handle, send_handle 
    import socket
    def worker(in_p, out_p): 
        '''
        Listening to in_p using recv_handle for socket file descriptor
        Then listen to the socket. If there's a message coming, send it back
        '''
        out_p.close()
        while True: 
            fd = recv_handle(in_p) 
            print('CHILD: GOT FD', fd)
            # Here we're reopening the same socket. socket.fromfd() returns a copy of the socket
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM, fileno=fd) as client:
                while True:
                    msg = client.recv(1024)
                    if not msg:
                        break 
                    print('CHILD: RECV {!r}'.format(msg))
                    client.send(msg)

    def server(address, in_p, out_p, worker_pid):
        '''
        1. Close the input connector
        2. Set up server based on the address
            - IPv4 vs IPv6:
                - V4 is like 192.168... 32 bit address. (4 bytes)
                - V6 is like 3ff3. .... hex 128 bit address. (8 hex) Headers are simpler, too.
            - Connection-Oriented (TCP) vs Connection-less protocols (UDP)
                - TCP has proper start and termination handshakes. 
                - UDP does not have such handshakes. Each data packet might follow a different path to reach the destination
                    - So data might be lost, or come in different orders
            
        3. Start listening to the socket. Once getting a client, it will close the connection?
            - Inter-Process Communication (IPC) uses: 
                1. files,
                2. socket
                3. Unix socket
                4. Memory mapped file
        4. Then, it sends the file descriptor of the client socket to another worker process.
            - using send_handle()
        4. On another terminal, start telnet. 
            - command: telnet ADDRESS port
            - Then you can send messages to the server
            - to quit, hit ctrl+']' to escape to telnet, and hit q to quit
        '''
        in_p.close() 
        # AF_INET means IpV4, SOCK_STREAM means TCP
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # REUSEADDR means the socket can be reused, even after it's unintentionally closed
        # Otherwise, if try reusing a closed socket, you see 'Address already in use'
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        s.bind(address)
        s.listen(1)
        while True:
            # blocking, return a new socket object, and client address
            client, addr = s.accept()
            print('SERVER: Got connection from', addr)
            # send a handle to out_p, to the target process pid?
            send_handle(out_p, client.fileno(), worker_pid)
            # Why closing client? Because we're reopening the socket object in the worker process
            # See above
            print("handled sent")
            client.close()
            print("client closed")

    c1, c2 = multiprocessing.Pipe()
    worker_p = multiprocessing.Process(target=worker, args=(c1,c2))
    worker_p.start()
    server_p = multiprocessing.Process(target=server, args=(('', 15000), c1, c2, worker_p.pid))
    server_p.start()

    c1.close()
    c2.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    # interesting, -- does make a difference
    parser.add_argument("--client", action="store_true", default=False, required=False)
    args = parser.parse_args()
    if args.client:
        print("client")
    else:
        print("server")
    
    
    # test_socket_file_descriptor_passing()


