# FICHEIROS ADICIONADOS

## A- Tarefa para leitura e envio dos dados do sensor (LT1000) para o DUNE (LTPanda)
  * dune/source/etc/ceov-asv.ini            -> contém info do catamarã X-2601
  * src/Sensors/LT1000                      -> pasta com tarefa para comunicar com o LT1000
  * src/Simulators/LT1000                   -> pasta que contém task do simulador

## B- Tarefa de Controlo PID para direção e velocidade
  * dune/source/etc/ceov-asv.ini            -> contém os valores de PID usados bem como outras funcionalidades
  * src/Control/USV/SimpleHeadingAndSpeed   -> tarefa de controlo PID
  * src/Navigation/USV/Waypoints            -> tarefa de teste unitário
  
## B- Envio de informação de atuação para o Arduino
  * dune/source/etc/ceov-asv.ini            -> contém a porta do Arduino e baudrate a utilizar
  * src/Actuators/UART_Comm                 -> tarefa de envio de mensagens para o Arduino
  * src/Actuators/Value_Generator           -> tarefa de teste unitário
  
## B- Controlo de Altitude Mínima
  * dune/source/etc/ceov-asv.ini            -> contém a altitude bem como tolerâncias de altitude
  * src/Supervisors/MinAltitude             -> tarefa de controlo de altitude mínima

## C- Receção de mensagens LoRa
  * dune/source/etc/ceov-asv.ini            -> contém a porta do módulo LoRa e baudrate a utilizar
  * src/Transports/Lora_serial              -> tarefa de receção de mensagens LoRa
