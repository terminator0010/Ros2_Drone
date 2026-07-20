import requests
import time
import logging

class TerrainElevationReader:
    def __init__(self, url="http://localhost:8080/json/position/ground-elev-m"):
        self.url = url
        self.last_elevation = 0.0
        
    def get_elevation(self):
        """
        Lê a elevação do terreno a partir da Property Tree do FlightGear.
        Timeout muito curto para não travar a simulação física.
        """
        try:
            # O timeout de 0.05s previne congelamentos na física (RK4)
            response = requests.get(self.url, timeout=0.05)
            if response.status_code == 200:
                data = response.json()
                # O FlightGear geralmente retorna o valor em data['value']
                # ao aceder a um endpoint JSON específico de propriedade
                if 'value' in data:
                    self.last_elevation = float(data['value'])
        except (requests.exceptions.RequestException, ValueError, KeyError):
            # Se falhar por timeout, conexão recusada, ou erro de parse, mantém a última elevação conhecida
            pass
            
        return self.last_elevation
