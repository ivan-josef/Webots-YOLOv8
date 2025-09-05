# 🤖 Pipeline de Visão Computacional e Controle de Robôs com Webots + ROS 2 + YOLOv8

Este projeto demonstra um pipeline de visão computacional e controle de robôs integrado, utilizando a simulação **Webots** e o framework **ROS 2**.  
Ele permite que um robô virtual se mova e use uma câmera para detectar objetos com o modelo **YOLOv8** em tempo real.

---

## 🚀 Funcionalidades

- **Simulação Realista**: Utiliza o simulador de robótica **Webots** para criar um ambiente virtual e um robô móvel, o **AUREA**.  
- **Comunicação ROS 2**: Estabelece uma conexão bidirecional entre a simulação Webots e a rede ROS 2, permitindo o fluxo de dados dos sensores e o envio de comandos de controle.  
- **Detecção de Objetos com YOLOv8**: Um nó ROS 2 dedicado processa o fluxo de vídeo da câmera do robô para detectar objetos e desenhar caixas delimitadoras (*bounding boxes*).  
- **Controle de Teleop**: Controla o movimento do robô e o pan da câmera usando comandos de teclado.  
- **Visualização em Tempo Real**: A imagem processada é publicada em um tópico para visualização externa com o `rqt_image_view`.  

---

## 🛠 Tecnologias e Dependências

- **ROS 2 Humble**: Middleware de comunicação robótica.  
- **Webots**: Ambiente de simulação de robótica.  
- **Python 3.10**: Linguagem principal dos nós.  
- **YOLOv8 & Ultralytics**: Framework de detecção de objetos.  
- **OpenCV & cv_bridge**: Processamento de imagem e integração ROS ↔ OpenCV.  

---

## 📂 Estrutura do Projeto

O projeto possui dois pacotes principais ROS 2:

- **`my_package`**  
  - Contém o driver do robô para Webots (`my_robot_driver.py`) e arquivos de configuração.  
  - Gerencia a conexão com a simulação.  

- **`object_finder`**  
  - Contém o nó de visão computacional (`connecting_and_showing.py`).  
  - Utiliza o YOLOv8 para detecção de objetos.  
  - Publica as *bounding boxes* e imagens processadas.  

---

## ⚙️ Como Executar

### 🔹 Pré-requisitos
- **Webots** instalado (versão compatível com ROS 2 Humble).  
- **ROS 2 Humble** instalado e configurado.  
- Um **workspace ROS 2** (ex.: `~/ros2_ws`).  

---

### 1. Clonar o Repositório
Dentro da pasta `src` do seu workspace ROS 2:
```bash
cd ~/ros2_ws/src
git clone <URL_DO_REPOSITORIO>
