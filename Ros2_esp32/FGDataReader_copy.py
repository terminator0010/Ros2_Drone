import socket

class FGDataReader:
    """
    Centraliza o recebimento de dados vindos do FlightGear via UDP (protocolo genérico).
    """
    def __init__(self, udp_ip="127.0.0.1", udp_port=5590):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        
        self.ground_elev = 0.0
        self.rudder = 0.0
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False) # Fundamental para não travar o loop físico
        
    def update(self):
        """
        Lê todos os pacotes UDP no buffer até o mais recente.
        """
        try:
            while True:
                data, _ = self.sock.recvfrom(1024)
                parts = data.decode('utf-8').strip().split(',')
                if len(parts) >= 2:
                    self.ground_elev = float(parts[0])
                    self.rudder = float(parts[1])
        except BlockingIOError:
            pass # Sem dados novos neste tick
        except Exception:
            pass # Falha de conversão/pacote quebrado, ignorar
            
    def get_elevation(self):
        return self.ground_elev
        
    def get_rudder(self):
        return self.rudder
        
    def close(self):
        self.sock.close()
