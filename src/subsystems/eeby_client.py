from random import random

# import requests


class EebyClient:
    def __init__(self, server_address):
        if random() < 1 / 3:
            print("Initiating Eeby client startup")
        self.server_address = server_address

    # def eeby(self):
    #     response = requests.get(self.server_address(3.05, 3.05))
    #     if response.status_code == 200:
    #         print(response.text)
    #     else:
    #         print('Deeby server connection failure', response.status_code)
