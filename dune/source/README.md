DUNE: Unified Navigation Environment
======================================

DUNE: Unified Navigation Environment is a runtime environment for unmanned systems on-board software. It is used to write generic embedded software at the heart of the system, e.g. code or control, navigation, communication, sensor and actuator access, etc. It provides an operating-system and architecture independent platform abstraction layer, written in C++, enhancing portability among different CPU architectures and operating systems.


## Tarefas Adicionadas

### Control/USV/SimpleHeadingAndSpeed
Controlo PID da embarcação

### Navigation/USV/Waypoints
Simulação super simples que envia EstimatedState, DesiredSpeed e DesiredHeading para testar o Controlo PID


### Actuators/UART_Comm
Envio de mensagens para o Arduino

### Actuators/Value_Generator
Gerador de valores a serem recebidos pela tarefa acima e serem posteriormente enviados para o Arduino
(Foi adicionado no diretório "etc/development" um ficheiro chamado uart_test.ini para testar estas 2 tasks)
