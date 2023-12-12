# Classificação de digitos manuscristos utilizando redes neurais Convolucionais

## Descrição

Classificação de imagens de dígitos manuscritos em 10 classes (0-9) utilizando uma rede neural convolucional. O dataset utilizado é o MNIST, que consiste em imagens de números manuscritos em escala de cinza com tamanho 28x28. 

## Pre processando o dataset

Nesta parte estamaos preparando os dados após importar o dataset **mnist**. A ordem do preprocessamento:
* Divisão dos dados em Teste e Treinamento
* Escalona as imagens entre 0 e 1
  * Os valores dos pixels estão representados em um range de 0 a 255, que é associado entre Preto (0) e Branco (255). Por isso devemos transformamos para float e dpois dividir por 255
* Garantir que cada imagem possui a resolução correta (28x28 pixels)
* Conversão de vetores para matrizes binarias

## Configurando a Rede Neural

Aqui estamos constuindo nosso modelo e definindo nossas camadas, além de suas configurações. Estamos usando 2 camadas de convolução, cada uma com um filtro de 32 e 64, respectivamente, representando a dimensão do output, além de ambas com um tamanho da janela de convolução (kernel_size) de 3x3. Cada uma dessas camadas é seguida por uma camada de pooling, que reduz o tamanho espacial da representação

## Treinamento da rede neural convolucional

Para o treinamento usamos um batch size de 128 e 3 epochs, como pedido na atividade

## Resultados

Abaixo verificamos o resultado e garantimos que tivemos uma acuracia superior a pedida na atividade
```
Test loss: 0.0464
Test accuracy: 0.985
```

## Video Demonstração 

Abaixo está a demonstração:

<div style="position:relative;width:fit-content;height:fit-content;">
            <a style="position:absolute;top:20px;right:1rem;opacity:0.8;" href="https://clipchamp.com/watch/C8V0jCRckaH">
                <img loading="lazy" style="height:22px;" src="https://youtu.be/MOEPZC82HQM" alt="Video no youtube" />
            </a>
</div>
