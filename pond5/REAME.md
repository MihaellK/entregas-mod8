## Chatbot e LLM pt2 - Ponderada 5

Parte 2 da ponderada 4, Chatbot e LLM. Agora houve a adição de um documento de contexto para o modelo. Neste documento há instruções para segurança do trabalho (workshop_rules.txt)

## Instalações Necessárias


Caso não tenha instalado as dependencias da pt1, siga os comandos abaixo:

```
curl https://ollama.ai/install.sh | sh
```

Também devemos instalar o gradio para nossa interface gráfica

```
pip install gradio 
```

A partir das regras definidas no arquivo 'Modelfile' o ollama criara o modelo especifico do nosso Engenheiro de Segurança, rode o comando:

```
ollama create safety -f Modelfile
```

## Executando

Para executar o chatbot, rode o comando abaixo:

```
python3 ./ollama-lang.py
```

Acessando o link fornecido no terminal: http://127.0.0.1:7860

Digite no campo de input seu prompt e aguarde a resposta do modelo

Resposta do modelo:
![EPIS necessarios](https://github.com/MihaellK/entregas-mod8/blob/main/pond5/media/Screenshot%20from%202023-11-29%2017-50-20.png?raw=true)

## Vídeo

![Vídeo da execução e do envio do prompt](https://youtu.be/sSRUX8YWsI8)
