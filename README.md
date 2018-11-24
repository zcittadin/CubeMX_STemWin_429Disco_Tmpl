# CubeMX_STemWin_429Disco_Tmpl

Template para placa STM32F429 Discovery utilizando CubeMX e biblioteca gráfica STemWin. Estrutura gerada para Atollic TrueSTUDIO.

Este projeto é um template "Hello World!" configurado através da ferramenta CubeMX para a placa STM32F429 Discovery.

Para utilizar basta clonar ou baixar este repositório, em seguida importar o projeto para seu workspace, executar o build da aplicação
e carregar no dispositivo.

O arquivo Cube_STemWin_429DISC.ioc pode ser aberto através do CubeMX e configurado para gerar o projeto em outras estruturas compatíveis,
por exemplo, com o Keil e o IAR.

O objetivo é ter um ponto de partida para outros projetos. Alterações podem ser feitas atuando nas cofigurações, via CubeMX, para que o projeto seja ajustado visando possibilitar a execução de demais tarefas, adicionando assim, outras funcionalidades ao projeto.

Foi utilizado FreeRTOS e CubeMX versão 5.0.

É possível utilizar a ferramenta "Drag and Drop" GUIBuilder, oficialmente distribuído junto a biblioteca STemWin. Esta ferramenta possibilita a construção da UI de forma intuitiva. 
Para tal, construa uma tela de aplicação no GUIBuilder, obedecendo a resolução do seu display. Adicione widgets, altere cores, etc... Ao salvar é gerado um código de GUI (Graphic User Interface), resultante do GUIBuilder, importe o arquivo gerado, com extensão ".c" para a pasta /STemWin/App, e chame a função CreateWindow() dentro da task responsável pela renderização da GUI, esta task normalmente está implementada no arquivo /STemWin/APP/GUI_App.c.
