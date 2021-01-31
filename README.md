# Sistema de planeamento e controlo de execução para USV

## Estado do Projeto

### Execução de Missões
A embarcação é capaz de executar missões que sejam conjuntos de waypoints. Estas são recebidas por WiFi do Neptus para a embarcação (DUNE) e a embarcação envia o seu estado atual para o Neptus. O USV executa em modo de simulação, em que o simulador VSIM do LSTS simula o envio de mensagens de estado (velocidade, localização e orientação) e consome as mensagens de atuação da embarcação simulando o seu efeito no sistema. O USV executa em modo "Hardware", isto é, sem simulação, sendo que o seu desempenho se encontra por testar no USV físico.

### Interfaces de Comunicação Neptus-DUNE
Apenas se recorre a WiFi para a comunicação entre o USV e a estação de controlo. Não se conseguiu implementar Iridium. A comunicação por LoRa apenas está em início de desenvolvimento como prova de conceito: a embarcação recebe mensagens arbitrárias, mas ainda não as processa como missões/mensagens IMC/text commands.

### Próximos Passos
* Iridium: implementar tarefa de comunicação capaz de receber e/ou transmitir mensagens por Iridium
* LoRa: modificar tarefa atual para interpretar mensagens LoRa recebidas como missões/mensagens IMC/text commands enviando essas mensagens para o resto da embarcação sob o formato IMC correto
* Gestão de comunicações: gerir a interface utilizada entre WiFi, Iridium ou LoRa<sup>[1](#myfootnote1)</sup>.
* Teste no X-2601: realizar teste de sistema, utilizando o X-2601.

## Hierarquia de Pastas do Repositório

### Engenharia
Contém todos os documentos e código fornecidos pelo Segundo Tenente Pedro Castro Fernandes, inclusive datasheets de material utilizado, e código de software utilizado no LattePanda e Arduino.

### arduino_dune
Contém o código utilizado no Arduino para receber mensagens do LattePanda e convertê-las para atuação nos lemes e motores.

### dune
Contém código do DUNE - software que corre a bordo dos sistemas do LSTS. Contém tarefas que não são utilizadas neste projeto especificamente, mas que permanecem como caso de estudo para desenvolvimento de novo código.

### neptus
Contém código e configurações do Neptus - software de controlo e monitorização de missões do LSTS. Contém o endereço único utilizado para o USV da marinha (denominado CEOV-ASV na aplicação).



<a name="myfootnote1">1</a>: a abordagem inicial que foi definida por equipa seria a utilização e tempos de resposta como fator de decisão. As tarefas de comunicação de Iridium e LoRa seriam tarefas "ativáveis" em que apenas uma delas estaria, a qualquer momento, ativa. Se a tarefa de WiFi demorasse demasiado a comunicar (timeout) passaria para a tarefa de LoRa, e da de LoRa para Iridium em último caso. Esta abordagem revelou-se mais complicada do que inicialmente previsto, pois a componente WiFi está "embutida" no DUNE e é necessário primeiro perceber como seria possível fazer esta gestão.
