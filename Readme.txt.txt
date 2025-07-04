Para executar o programa:
	1 - execute renode str-renode.resc
	2 - Compile o programa no modo Debug utilizando a IDE stmCube32 
	3 - Coloque um breakpoint na linha 160 do arquivo Core/Src/miros.cpp
	4 - Execute o programa no modo debug
	5 - No Menu Expressions, analise a o buffer queue.buffer
	6 - Aperte o botão resume para avançar um tick
	
Cada Thread colocará um número neste buffer de acordo com seu período e prioridade,
As threads aperiódicas são colocadas em uma fila pela função Tick de acordo com um
período e um desvio de tempo aleatório.

Foi implementado BS (Background Server) e o protocolo NPP.
