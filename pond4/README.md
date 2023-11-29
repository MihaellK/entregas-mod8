## Chatbot e LLM - Ponderada 4

Nesta atividade foi desenvolvido um chatbot que utiliza LLM através da biblioteca Ollama. O modelo foi setado para interpretar um técnico em segurança do trabalho, auxiliando assim trabalhadores com duvidas de operações industrias

## Instalações Necessárias

Primeiro devemos instalar o ollama executando o comando abaixo:

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

## Vídeo

[![Vídeo da execução e do envio do prompt](https://www.youtube.com/watch?v=UXF2CZgf1xA)]

Resultado após 10 minutos de exec 

