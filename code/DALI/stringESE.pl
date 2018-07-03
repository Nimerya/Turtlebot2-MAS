:- use_module(library(lists)).

:- dynamic ris/1.
:- assert(ris(0)).


sostituisci(Tolgo, Metto) :- % Sostituisce Tolgo con Metto dentro la lista memorizzata nella variabile dynamic ris
	ris(List),
	( select(Tolgo, List, Metto, Out) ->  retract(ris(_)), print(Out), assert(ris(Out));
	 print(';')).

pulisciSpazi(L) :- % Ripristina gli spazi
	(foreach(Elem, L) do 
		(Elem is 79 -> print(Elem), sostituisci(79, 32); % 79 è O in ASCII, 32 è lo spazio in ASCII
		print('...'))
	).

pulisciParTonAp(L) :- % Ripristina le (
	(foreach(Elem, L) do 
		(Elem is 65 -> print(Elem), sostituisci(65, 40); % 65 è A in ASCII, 40 è la parentesi tonda aperta in ASCII
		 print('...'))
	).

pulisciParTonCh(L) :- % Ripristina le )
	(foreach(Elem, L) do 
		(Elem is 66 -> print(Elem), sostituisci(66, 41); % 66 è B in ASCII, 41 è la parentesi tonda chiusa in ASCII
		 print('...'))
	).

pulisciParQuAp(L) :- % Ripristina le [
	(foreach(Elem, L) do 
		(Elem is 67 -> print(Elem), sostituisci(67, 91); % 67 è C in ASCII, 91 è la parentesi quadra aperta in ASCII
		 print('...'))
	).

pulisciParQuCh(L) :- % Ripristina le ]
	(foreach(Elem, L) do 
		(Elem is 68 -> print(Elem), sostituisci(68, 93); % 68 è D in ASCII, 93 è la parentesi quadra chiusa in ASCII
		 print('...'))
	).

pulisciPunti(L) :- % Ripristina i punti
	(foreach(Elem, L) do 
		(Elem is 69 -> print(Elem), sostituisci(69, 46); % 69 è E in ASCII, 46 è il punto in ASCII
		print('...'))
	).

pulisciVirgole(L) :- % Ripristina le virgole
	(foreach(Elem, L) do 
		(Elem is 70 -> print(Elem), sostituisci(70, 44); % 70 è F in ASCII, 44 è la virgola in ASCII
		print('...'))
	).

pulisciSlash(L) :- % Ripristina gli slash
	(foreach(Elem, L) do 
		(Elem is 71 -> print(Elem), sostituisci(71, 47); % 71 è G in ASCII, 47 è lo slash in ASCII
		print('...'))
	).

pulisciBackSlash(L) :- % Ripristina gli slash
	(foreach(Elem, L) do 
		(Elem is 72 -> print(Elem), sostituisci(72, 92); % 72 è H in ASCII, 92 è lo back-slash in ASCII
		print('...'))
	).

pulisciApici(L) :- % Ripristina gli apici
	(foreach(Elem, L) do 
		(Elem is 73 -> print(Elem), sostituisci(73, 39); % 73 è I in ASCII, 39 è apice in ASCII
		print('...'))
	).

pulisciStringa(At, Z) :-
	name(At, L),
	retract(ris(_)),
	assert(ris(L)),
	pulisciSpazi(L), pulisciParTonAp(L), pulisciParTonCh(L), pulisciParQuAp(L), pulisciParQuCh(L),
	pulisciPunti(L), pulisciVirgole(L), pulisciSlash(L), pulisciBackSlash(L), pulisciApici(L),
	ris(X), name(Z,X), print(Z).


