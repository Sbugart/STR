Para executar o programa:
	1 - execute renode str-renode.resc
	2 - Compile o programa no modo Debug utilizando a IDE stmCube32 
	3 - Coloque um breakpoint na linha 102 do arquivo Core/Inc/miros.cpp
	4 - Execute o programa no modo debug
	5 - No Menu Expressions, analise a o buffer queue.buffer
	6 - Aperte o botão resume para avançar um tick
	
Cada Thread colocará um número neste buffer de acordo com seu período e prioridade
