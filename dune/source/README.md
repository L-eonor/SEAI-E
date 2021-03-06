DUNE: Unified Navigation Environment
======================================

DUNE: Unified Navigation Environment is a runtime environment for unmanned systems on-board software. It is used to write generic embedded software at the heart of the system, e.g. code or control, navigation, communication, sensor and actuator access, etc. It provides an operating-system and architecture independent platform abstraction layer, written in C++, enhancing portability among different CPU architectures and operating systems.


## Tarefas Adicionadas

### [Control/USV/SimpleHeadingAndSpeed](src/Control/USV/SimpleHeadingAndSpeed)
Controlo PID da embarcação

### [Navigation/USV/Waypoints](src/Navigation/USV/Waypoints)
Simulação super simples que envia EstimatedState, DesiredSpeed e DesiredHeading para testar o Controlo PID. Foi adicionada no ficheiro [test_pid.ini](etc/development/test_pid.ini) para testar a tarefa de PID.


### [Actuators/UART_Comm](src/Actuators/UART_Comm)
Envio de mensagens para o Arduino

### [Actuators/Value_Generator](src/Actuators/Value_Generator)
Gerador de valores a serem recebidos pela tarefa acima e serem posteriormente enviados para o Arduino
(Foi adicionado um ficheiro chamado [uart_test.ini](etc/development/uart_test.ini) para testar estas 2 tasks)
