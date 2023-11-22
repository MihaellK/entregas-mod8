import rclpy
from rclpy.node import Node
import re

class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot_node')

    def go_almox(self):
        return "Indo para o almoxarifado!"
        
    def go_packaging(self):
        return "Indo para o setor Packaging!"
        
    def return_almox(self):
        return "Retornando ao almoxarifado!"
        
    def return_packaging(self):
        return "Retornando ao setor Packaging!"

    def go_to(self, place=None):
        if (place):
            return f"Estou indo para {place}!"
        else:
            return f"Estou indo para um lugar não identificado :("

    def back_to(self, place=None):
        if (place):
            return f"Estou voltando para {place}!"
        else:
            return f"Estou voltando Casa!"
    
    intent_dict = {
        r"\b(?:(?:(?:[Mm]e)?\s?(?:[Vv]a[i]?|[Mm]ostre|[Ll]eve))\s(?:para|pra|pro)?\s?(?:o)?\s?(?:setor)?\s?(?:de)?\s?([Aa]lmox(?:arifado)|packaging|distribuição|armazenamento|casa)\s?)": "goto",
        r"\b(?:[Rr]etorn[ea]|[vV]olt[eao])\s(?:[Pp]a?r?[oa]|ao)?\s?(?:o|a)?\s?([Aa]lmox(?:arifado)|packaging|distribuição|armazenamento|casa)":"backto",
    }

    action_dict = {
        "goto": go_to,
        "backto": back_to,
    }

    def chat_loop(self):
        while rclpy.ok():
            command = input("Escreva o lugar que deseja ir: ")
            for key, value in self.intent_dict.items():
                pattern = re.compile(key)
                groups = pattern.findall(command)
                if groups:
                    print(f"{self.action_dict[value](self,groups[0])}", end=" ")
            print()


def main(args=None):
    rclpy.init(args=args)
    chatbot_node = ChatbotNode()
    chatbot_node.chat_loop()
    rclpy.spin(chatbot_node)
    chatbot_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

    