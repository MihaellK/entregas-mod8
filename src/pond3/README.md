## Desenvolvimento de Chatbot - Ponderada 3

### Como Rodar
Após abrir a pasta no terminal garanta que voce possui todas as dependencias instalas com: 
```
sudo apt install python3-rosdep
```
e logo
```
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

para buildar nosso pacote, necessitamos rodar e instalar com o comando abaixo:
```
sudo apt install python3-colcon-common-extensions
```
Build com o seguinte comado
```
colcon build
```
COnfigure o workspace dando o source abaixo

```
source install/local_setup.bash #se estiver usando zsh, mude para setup.zsh
```
e finalmente rode com o comando:
```
ros2 run pond3 chatbot
```

Agora só precisa dar os comandos de ir ou voltar para diferentes setores, como, Almoxarifado, Packaging, Distribuição, Armazenamento e Casa, sendo o ponto 0 do Robo.
