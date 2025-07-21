## 👨‍💻 Autoria

Projeto desenvolvido por:

- **Ialy Sousa**  
- **Estevão Holanda**  
- **Eduardo Nogueira**  
- **Wesley Wilson**

Curso de Engenharia de Computação – 2025  
Instituto Federal da Paraíba (IFPB)

---
# 🎮 Jogo Interativo com ESP32, Acelerômetro, OLED e SD Card

Projeto de sistema embarcado que integra múltiplos periféricos para criar uma plataforma de minigames controlados por movimento. Desenvolvido com **ESP32 + FreeRTOS** no framework **ESP-IDF**.

---

## 📌 Funcionalidades

- 🕹️ **Menu de Seleção** com 4 minigames
- 🧭 **Controle por Movimento** usando MPU6050 (acelerômetro e giroscópio)
- 📺 **Interface Gráfica Monocromática** com display OLED SSD1306
- 💾 **Armazenamento Persistente** de recordes em cartão SD (SPI)
- 🔊 **Efeitos Sonoros** via Buzzer (PWM)
- 🔘 **Navegação com Botões Físicos**
- ⚙️ **Estrutura baseada em Máquina de Estados + Multithreading (FreeRTOS)**

---

## 🔧 Componentes Utilizados

| Componente      | Interface | Função                                       |
|-----------------|-----------|----------------------------------------------|
| ESP32           | MCU       | Controle geral do sistema                    |
| MPU6050         | I2C       | Sensor inercial (acelerômetro/giroscópio)   |
| SSD1306         | I2C       | Display OLED para gráficos monocromáticos    |
| SD Card Module  | SPI       | Armazenamento de recordes                    |
| Buzzer          | PWM       | Sons e feedback sonoro                       |
| Botões (x2)     | GPIO      | Navegação e seleção no menu                  |

---

## 📐 Diagrama de Blocos

![Diagrama de Blocos](/path/to/bloco_diagram.png)

---

## 📝 Diagrama Esquemático

![Diagrama Esquemático](/Diagrama_Esquematico.pdf)

---

## 🛠️ Layout de Montagem (Protoboard)

![Layout Físico](/path/to/layout_fisico.png)

---

## 🔄 Fluxo de Execução

![Fluxograma](/path/to/fluxo_maquina_estados.png)

---

## 🚀 Como Executar

### Pré-requisitos:
- ESP32 DevKit
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) instalado
- Python 3, toolchain configurado

### Passos:

```bash
# Clone o repositório
git clone https://github.com/seu-usuario/nome-do-repositorio.git
cd nome-do-repositorio

# Configure o ambiente
idf.py set-target esp32

# Compile o projeto
idf.py build

# Flash no dispositivo
idf.py -p /dev/ttyUSB0 flash

# Monitor serial
idf.py -p /dev/ttyUSB0 monitor
