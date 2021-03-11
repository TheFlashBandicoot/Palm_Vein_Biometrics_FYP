

# --------------------------------------- Description --------------------------------------- #

# Python image identification protocol. This python file contains the server and client 
# classes, used to send and receive image data for enrollment or identification. 

# Author: Bryan McSweeney 26/09/20
# Version: 1.2

# ------------------------------------- Imports & global variables -------------------------------------- #

import socket  

header_fixed_len = 8    # fixed length of headers in bytes
buffer_size = 4096  # size of recv buffer in bytes

# ------------------------------------------------- Server Class Definition ------------------------------------------------- #

class I2P_Server:

    global header_fixed_len 
    global buffer_size

    # -------------------------------------------------------------------------------------- #
    # Creates socket object for instance, binds to a ('host',port) tuple and listens on port.
    # -------------------------------------------------------------------------------------- #
    def __init__(self, host, port):

        try:
            self.I2P_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	# creates socket object (note: SOCK_STREAM means TCP)
        except socket.error as error:
            self.shutdown() # destroy socket
            raise type(error)('Error occurred while creating the socket: \n\t%s' % error) from error  # re-raise error with custom message so application can react.

        try:
            self.I2P_sock.bind((host,port))	# try binding to port
        except socket.error as error:
            self.shutdown() # destroy socket
            raise type(error)('Error occurred when binding to port %d: \n\t%s' % (port, error)) from error  # re-raise error with custom message so application can react.

        self.I2P_sock.listen(5)	# listen on socket, allow backlog of up to 5 requests.

    # -------------------------------------------------------------------------------------- #
    # Accepts connection (blocking wait). Prints address of connected client.
    # -------------------------------------------------------------------------------------- #
    def accept(self):

        self.conn, self.addr = self.I2P_sock.accept()   # creates client socket object 'conn'
        print('Connected to ', self.addr)   # prints address of connected client

    # -------------------------------------------------------------------------------------- #
    # Method to receive data (blocking wait). returns header and/or data
    # -------------------------------------------------------------------------------------- #
    def receive(self):
    
        self.conn.settimeout(10)    # set recv to time-out after 10 seconds of no transmission

        try:
            recv_buffer = self.conn.recv(buffer_size)   # waits for data. saves data to a buffer variable
        except socket.timeout as timeout:   # catch timeout exception, reset timeout value, and re-raise exception for application program to handle.
            self.conn.settimeout(None)
            raise type(timeout)('recv timed out. No data recevied from client: \n\t%s' % timeout) from timeout

        if recv_buffer == b'':  # if client closes connection, they will send b''.
            self.close_connection() # close connection on server end
            raise socket.error('Client closed connection.') # raise exception to alert application.

        while len(recv_buffer) < header_fixed_len: # keep receiving till we have enough bytes for the header
            try:
                recv_buffer = recv_buffer + self.conn.recv(buffer_size) # append received data to buffer
            except socket.timeout as timeout:   # catch timeout exception, reset timeout value, and re-raise exception for application program to handle.
                self.conn.settimeout(None)
                raise type(timeout)('recv timed out. Incomplete header received from client: \n\t%s' % timeout) from timeout

        header = recv_buffer[:header_fixed_len] # isolate header
        header = header.decode()    # decode header from utf-8 to string

        if header.isnumeric():  # if header is only numbers, assume it contains the file size of some data to follow
            file_size = int(header)
            data = recv_buffer[header_fixed_len:]   # save the rest of the buffer to data

            while len(data) < file_size:    # keep receiving until 'data' size = file_size
                try:
                    recv_buffer = self.conn.recv( min(buffer_size, ( file_size - len(data) )) ) # receive in chunks of size 'buffer_size' unless the amount left to receive is less
                except socket.timeout as timeout:   # catch timeout exception, reset timeout value, and re-raise exception for application program to handle.
                    self.conn.settimeout(None)
                    raise type(timeout)('recv timed out. Incomplete data received from client \n\t%s' % timeout) from timeout

                data = data + recv_buffer

            self.conn.settimeout(None)
            return data # no need to return header.

        else:
            self.conn.settimeout(None)
            return header   # if header is not numeric, it must be a command and must be returned here.

    # -------------------------------------------------------------------------------------- #
    # Method to transmit data. Will send header of fixed length, and data if header is int
    # -------------------------------------------------------------------------------------- #
    def transmit(self, header, data): 

        if type(header) == str:     # check if header is a string
            header = header.encode()    # encode to utf-8

            if len(header) <= header_fixed_len:  # if within header limit, continue
                while len(header) < header_fixed_len:
                    header = b'\x20' + header   # add space character to front of header until reaches 'header_fixed_len'

                try:
                    self.conn.sendall(header)
                except socket.error as error:
                    raise type(error)('Error occurred while sending header: \n\t%s' % error) from error  # re-raise error with custom message so application can react.

                print('command transmitted successfully')   # if no errors in sending, report successful (NOTE: no guarantee data was actually sent, just sure of no errors)
                
            else:       # if outside limit, raise ValueError exception.
                raise ValueError('Length of encoded header exceeded protocol limit')    # raise error to be handled by application

        elif type(header) == int:   # check if header is an int

            if type(data) != bytes: # check data is of valid type, and encode to utf-8 if necessary

                if type(data) == int:
                    data = (str(data)).encode()
                elif type(data) == str:
                    data = data.encode()
                else:
                    raise TypeError('data of incorrect type. must be bytes, int or str')

            if (len( (str(header)).encode() ) <= header_fixed_len) and (len(data) == header): # check that header is within limit AND describes length of data
                header = ( str(header) ).encode()

                while len(header) < header_fixed_len:
                    header = b'\x30' + header   # add '0' character to front of header until reaches 'header_fixed_len'
                
                data = header + data    # concatenate header and data

                try:
                    self.conn.sendall(data)
                except socket.error as error:
                    raise type(error)('Error occurred while sending data: \n\t%s' % error) from error  # re-raise error with custom message so application can react.

                print('data transmitted successfully')  # if no errors in sending, report successful (NOTE: no guarantee data was actually sent, just sure of no errors)

            else:
                raise ValueError('Length of encoded header either exceeds limit, or does not describe the length of the data')  # raise error to be handled by application

        else:
            raise TypeError('Header of incorrect type. Must be str or int') # raise error to be handled by application

    # -------------------------------------------------------------------------------------- #
    # Method to close connection with client. This does not destroy the server socket.
    # -------------------------------------------------------------------------------------- #
    def close_connection(self):

        if self.conn != None:   # check that a connection exists
            self.conn.close()
            self.conn = None
            print('Closed connection to ', self.addr)
        else:
            pass    # if no connection exists, do nothing

    # ------------------------------------------------------------------------------------------ #
    # Method to destroy server socket (cleanup). ALWAYS RUN THIS IN FINALLY BLOCK! EVEN IF ERRORS
    # ------------------------------------------------------------------------------------------ #
    def shutdown(self):

        if self.I2P_sock != None:   # check that socket exists
            self.I2P_sock.close()
            self.I2P_sock = None
            print('Destroyed server socket')
        else:
            pass    # if socket does not exist, do nothing





# ------------------------------------------------- Client Class Definition ------------------------------------------------- #

class I2P_Client:

    # -------------------------------------------------------------------------------------- #
    # Creates socket object for instance
    # -------------------------------------------------------------------------------------- #
    def __init__(self):

        try:
            self.I2P_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	# creates socket object (note: SOCK_STREAM means TCP)
        except socket.error as error:
            self.shutdown() # destroy socket
            raise type(error)('Error occurred while creating the socket: \n\t%s' % error) from error  # re-raise error with custom message so application can react.
            
    # -------------------------------------------------------------------------------------------------- #
    # Initiates connection to specified host and port (blocking wait). Prints confirmation of connection.
    # -------------------------------------------------------------------------------------------------- #
    def connect(self, host, port):

        self.I2P_sock.settimeout(10)

        try:
            self.I2P_sock.connect( (host,port) )    # connects to server at specified IP and port number
        except socket.error as error:
            self.I2P_sock.settimeout(None)
            raise type(error)('Error occurred while connecting to %s : %d: \n\t%s' % (host, port, error) ) from error  # re-raise error with custom message so application can react.
        except socket.timeout as timeout:
            self.I2P_sock.settimeout(None)
            raise type(timeout)('connect timed out while attempting to connect to %s : %d: \n\t%s' % (host, port, timeout))

        self.I2P_sock.settimeout(None)
        print('Connected to %s : %d' % (host, port) )   # confirm connection if successful
                
    # -------------------------------------------------------------------------------------- #
    # Method to receive data (blocking wait). returns header and/or data
    # -------------------------------------------------------------------------------------- #
    def receive(self):
        
        self.I2P_sock.settimeout(10)

        try:
            recv_buffer = self.I2P_sock.recv(buffer_size)   # waits for data. saves data to a buffer variable
        except socket.timeout as timeout:   # catch timeout exception, reset timeout value, and re-raise exception for application program to handle.
            self.I2P_sock.settimeout(None)    
            raise type(timeout)('recv timed out. No data recevied from Server: \n\t%s' % timeout) from timeout

        if recv_buffer == b'':  # if server closes connection, they will send b''.
            self.shutdown() # shutdown connection on client end
            raise socket.error('Server closed connection.') # alert application via exception

        while len(recv_buffer) < header_fixed_len: # keep receiving till we have enough bytes for the header
            try:
                recv_buffer = recv_buffer + self.I2P_sock.recv(buffer_size) # append received data to buffer
            except socket.timeout as timeout:   # catch timeout exception, reset timeout value, and re-raise exception for application program to handle.
                self.I2P_sock.settimeout(None)
                raise type(timeout)('recv timed out. Incomplete header received from server: \n\t%s' % timeout) from timeout

        header = recv_buffer[:header_fixed_len] # isolate header
        header = header.decode()    # decode header from utf-8 to string

        if header.isnumeric():  # if header is only numbers, assume it contains the file size of some data to follow
            file_size = int(header)
            data = recv_buffer[header_fixed_len:]   # save the rest of the buffer to data

            while len(data) < file_size:    # keep receiving until 'data' size = file_size
                try:
                    recv_buffer = self.I2P_sock.recv( min(buffer_size, ( file_size - len(data) )) ) # receive in chunks of size 'buffer_size' unless the amount left to receive is less
                except socket.timeout as timeout:   # catch timeout exception, reset timeout value, and re-raise exception for application program to handle.
                    self.I2P_sock.settimeout(None)
                    raise type(timeout)('recv timed out. Incomplete data received from server \n\t%s' % timeout) from timeout

                data = data + recv_buffer

            self.I2P_sock.settimeout(None)
            return data # no need to return header.

        else:
            self.I2P_sock.settimeout(None)
            return header   # if header is not numeric, it must be a command and must be returned here.

    # -------------------------------------------------------------------------------------- #
    # Method to transmit data. Will send header of fixed length, and data if header is int
    # -------------------------------------------------------------------------------------- #
    def transmit(self, header, data):

        if type(header) == str:     # check if header is a string
            header = header.encode()    # encode to utf-8

            if len(header) <= header_fixed_len:  # if within header limit, continue
                while len(header) < header_fixed_len:
                    header = b'\x20' + header   # add space character to front of header until reaches 'header_fixed_len'
                try:
                    self.I2P_sock.sendall(header)
                except socket.error as error:
                    raise type(error)('Error occurred while sending command: \n\t%s' % error) from error  # re-raise error with custom message so application can react.
                
                print('command transmitted successfully')   # if no errors in sending, report successful (NOTE: no guarantee data was actually sent, just sure of no errors)
                
            else:       # if outside limit, raise ValueError exception.
                raise ValueError('Length of encoded header exceeded protocol limit')    # raise error to be handled by application

        elif type(header) == int:   # check if header is an int

            if type(data) != bytes: # check data is of valid type, and encode to utf-8 if necessary

                if type(data) == int:
                    data = (str(data)).encode()
                elif type(data) == str:
                    data = data.encode()
                else:
                    raise TypeError('data of incorrect type. must be bytes, int or str')

            if (len( (str(header)).encode() ) <= header_fixed_len) and (len(data) == header): # check that header is within limit AND describes length of data
                header = (str(header)).encode()

                while len(header) < header_fixed_len:
                    header = b'\x30' + header   # add '0' character to front of header until reaches 'header_fixed_len'

                data = header + data    # concatenate header and data

                try:
                    self.I2P_sock.sendall(data)
                except socket.error as error:
                    raise type(error)('Error occurred while sending data: \n\t%s' % error) from error  # re-raise error with custom message so application can react.

                print('data transmitted successfully')  # if no errors in sending, report successful (NOTE: no guarantee data was actually sent, just sure of no errors)

            else:
                raise ValueError('Length of encoded header either exceeds limit, or does not describe the length of the data')  # raise error to be handled by application

        else:
            raise TypeError('Header of incorrect type. Must be str or int') # raise error to be handled by application

    # -------------------------------------------------------------------------------------- #
    # Method to close connection with server. This destroys the socket.
    # -------------------------------------------------------------------------------------- #
    def shutdown(self):

        if self.I2P_sock != None:   # check that socket exists
            self.I2P_sock.close()
            self.I2P_sock = None
            print('Destroyed client socket')
        else:
            pass    # if no socket exists, do nothing

