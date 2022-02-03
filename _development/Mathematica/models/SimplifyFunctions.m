(* ::Package:: *)

(*
Expression -> espressione da semplificare
R-> Lista di matrici che sono matrici di rotazione ({R1,R2} per esempio)
customeq-> lista di vincoli/semplificazioni passate a mano (tipo {delta>=0 && delta appartenente a Reals})
Attento che ci stanno le fullsimplify quindi c'\[EGrave] il rischio che duri all'infinito
*) 
SimplifyRM[Expression_, R_] := (
   eq = Map[{{R[[#, 1, 1]], R[[#, 1, 2]], R[[#, 1, 3]], R[[#, 2, 1]], 
         R[[#, 2, 2]], R[[#, 2, 3]], R[[#, 3, 1]], R[[#, 3, 2]], 
         R[[#, 3, 3]]} \[Element] Reals, 
       R[[#, 1, 1]]^2 + R[[#, 1, 2]]^2 + R[[#, 1, 3]]^2 == 1, 
       R[[#, 2, 1]]^2 + R[[#, 2, 2]]^2 + R[[#, 2, 3]]^2 == 1, 
       R[[#, 3, 1]]^2 + R[[#, 3, 2]]^2 + R[[#, 3, 3]]^2 == 1, 
       R[[#, 1, 1]]^2 + R[[#, 2, 1]]^2 + R[[#, 3, 1]]^2 == 1, 
       R[[#, 1, 2]]^2 + R[[#, 2, 2]]^2 + R[[#, 3, 2]]^2 == 1, 
       R[[#, 1, 3]]^2 + R[[#, 2, 3]]^2 + R[[#, 3, 3]]^2 == 1, 
       R[[#, 1, 1]]*R[[#, 1, 3]] + R[[#, 2, 1]]*R[[#, 2, 3]] + 
         R[[#, 3, 1]]*R[[#, 3, 3]] == 0, 
       R[[#, 1, 1]]*R[[#, 1, 2]] + R[[#, 2, 1]]*R[[#, 2, 2]] + 
         R[[#, 3, 1]]*R[[#, 3, 2]] == 0, 
       R[[#, 1, 2]]*R[[#, 1, 3]] + R[[#, 2, 2]]*R[[#, 2, 3]] + 
         R[[#, 3, 2]]*R[[#, 3, 3]] == 0, 
       R[[#, 1, 1]]*R[[#, 2, 1]] + R[[#, 1, 2]]*R[[#, 2, 2]] + 
         R[[#, 1, 3]]*R[[#, 2, 3]] == 0, 
       R[[#, 1, 1]]*R[[#, 3, 1]] + R[[#, 1, 2]]*R[[#, 3, 2]] + 
         R[[#, 1, 3]]*R[[#, 3, 3]] == 0, 
       R[[#, 2, 1]]*R[[#, 3, 1]] + R[[#, 2, 2]]*R[[#, 3, 2]] + 
         R[[#, 2, 3]]*R[[#, 3, 3]] == 0, 
       R[[#, 2, 2]]*R[[#, 3, 3]] - R[[#, 2, 3]]*R[[#, 3, 2]] == 
        R[[#, 1, 1]], 
       R[[#, 2, 3]]*R[[#, 3, 1]] - R[[#, 2, 1]]*R[[#, 3, 3]] == 
        R[[#, 1, 2]], 
       R[[#, 2, 1]]*R[[#, 3, 2]] - R[[#, 2, 2]]*R[[#, 3, 1]] == 
        R[[#, 1, 3]], 
       R[[#, 1, 3]]*R[[#, 3, 2]] - R[[#, 1, 2]]*R[[#, 3, 3]] == 
        R[[#, 2, 1]], 
       R[[#, 1, 1]]*R[[#, 3, 3]] - R[[#, 1, 3]]*R[[#, 3, 1]] == 
        R[[#, 2, 2]], 
       R[[#, 1, 2]]*R[[#, 3, 1]] - R[[#, 1, 1]]*R[[#, 3, 2]] == 
        R[[#, 2, 3]], 
       R[[#, 1, 2]]*R[[#, 2, 3]] - R[[#, 1, 3]]*R[[#, 2, 2]] == 
        R[[#, 3, 1]], 
       R[[#, 1, 3]]*R[[#, 2, 1]] - R[[#, 1, 1]]*R[[#, 2, 3]] == 
        R[[#, 3, 2]], 
       R[[#, 1, 1]]*R[[#, 2, 2]] - R[[#, 1, 2]]*R[[#, 2, 1]] == 
        R[[#, 3, 3]], 
       R[[#, 2, 3]]*R[[#, 3, 2]] - 
         R[[#, 2, 2]]*R[[#, 3, 3]] == -R[[#, 1, 1]], 
       R[[#, 2, 1]]*R[[#, 3, 3]] - 
         R[[#, 2, 3]]*R[[#, 3, 1]] == -R[[#, 1, 2]], 
       R[[#, 2, 2]]*R[[#, 3, 1]] - 
         R[[#, 2, 1]]*R[[#, 3, 2]] == -R[[#, 1, 3]], 
       R[[#, 1, 2]]*R[[#, 3, 3]] - 
         R[[#, 1, 3]]*R[[#, 3, 2]] == -R[[#, 2, 1]], 
       R[[#, 1, 3]]*R[[#, 3, 1]] - 
         R[[#, 1, 1]]*R[[#, 3, 3]] == -R[[#, 2, 2]], 
       R[[#, 1, 1]]*R[[#, 3, 2]] - 
         R[[#, 1, 2]]*R[[#, 3, 1]] == -R[[#, 2, 3]], 
       R[[#, 1, 3]]*R[[#, 2, 2]] - 
         R[[#, 1, 2]]*R[[#, 2, 3]] == -R[[#, 3, 1]], 
       R[[#, 1, 1]]*R[[#, 2, 3]] - 
         R[[#, 1, 3]]*R[[#, 2, 1]] == -R[[#, 3, 2]], 
       R[[#, 1, 2]]*R[[#, 2, 1]] - 
         R[[#, 1, 1]]*R[[#, 2, 2]] == -R[[#, 3, 3]]} &, 
     Range[1, Dimensions[R][[1]]]];
   eq = Flatten[eq, 1];
   Return[FullSimplify[Expression, eq]];
   );

SimplifyRM[Expression_, R_, customeq_] := (
   eq = Join[
     Map[{{R[[#, 1, 1]], R[[#, 1, 2]], R[[#, 1, 3]], R[[#, 2, 1]], 
          R[[#, 2, 2]], R[[#, 2, 3]], R[[#, 3, 1]], R[[#, 3, 2]], 
          R[[#, 3, 3]]} \[Element] Reals,
        R[[#, 1, 1]]^2 + R[[#, 1, 2]]^2 + R[[#, 1, 3]]^2 == 1, 
        R[[#, 2, 1]]^2 + R[[#, 2, 2]]^2 + R[[#, 2, 3]]^2 == 1, 
        R[[#, 3, 1]]^2 + R[[#, 3, 2]]^2 + R[[#, 3, 3]]^2 == 1, 
        R[[#, 1, 1]]^2 + R[[#, 2, 1]]^2 + R[[#, 3, 1]]^2 == 1, 
        R[[#, 1, 2]]^2 + R[[#, 2, 2]]^2 + R[[#, 3, 2]]^2 == 1, 
        R[[#, 1, 3]]^2 + R[[#, 2, 3]]^2 + R[[#, 3, 3]]^2 == 1, 
        R[[#, 1, 1]]*R[[#, 1, 3]] + R[[#, 2, 1]]*R[[#, 2, 3]] + 
          R[[#, 3, 1]]*R[[#, 3, 3]] == 0, 
        R[[#, 1, 1]]*R[[#, 1, 2]] + R[[#, 2, 1]]*R[[#, 2, 2]] + 
          R[[#, 3, 1]]*R[[#, 3, 2]] == 0, 
        R[[#, 1, 2]]*R[[#, 1, 3]] + R[[#, 2, 2]]*R[[#, 2, 3]] + 
          R[[#, 3, 2]]*R[[#, 3, 3]] == 0, 
        R[[#, 1, 1]]*R[[#, 2, 1]] + R[[#, 1, 2]]*R[[#, 2, 2]] + 
          R[[#, 1, 3]]*R[[#, 2, 3]] == 0, 
        R[[#, 1, 1]]*R[[#, 3, 1]] + R[[#, 1, 2]]*R[[#, 3, 2]] + 
          R[[#, 1, 3]]*R[[#, 3, 3]] == 0, 
        R[[#, 2, 1]]*R[[#, 3, 1]] + R[[#, 2, 2]]*R[[#, 3, 2]] + 
          R[[#, 2, 3]]*R[[#, 3, 3]] == 0, 
        R[[#, 2, 2]]*R[[#, 3, 3]] - R[[#, 2, 3]]*R[[#, 3, 2]] == 
         R[[#, 1, 1]], 
        R[[#, 2, 3]]*R[[#, 3, 1]] - R[[#, 2, 1]]*R[[#, 3, 3]] == 
         R[[#, 1, 2]], 
        R[[#, 2, 1]]*R[[#, 3, 2]] - R[[#, 2, 2]]*R[[#, 3, 1]] == 
         R[[#, 1, 3]], 
        R[[#, 1, 3]]*R[[#, 3, 2]] - R[[#, 1, 2]]*R[[#, 3, 3]] == 
         R[[#, 2, 1]], 
        R[[#, 1, 1]]*R[[#, 3, 3]] - R[[#, 1, 3]]*R[[#, 3, 1]] == 
         R[[#, 2, 2]], 
        R[[#, 1, 2]]*R[[#, 3, 1]] - R[[#, 1, 1]]*R[[#, 3, 2]] == 
         R[[#, 2, 3]], 
        R[[#, 1, 2]]*R[[#, 2, 3]] - R[[#, 1, 3]]*R[[#, 2, 2]] == 
         R[[#, 3, 1]], 
        R[[#, 1, 3]]*R[[#, 2, 1]] - R[[#, 1, 1]]*R[[#, 2, 3]] == 
         R[[#, 3, 2]], 
        R[[#, 1, 1]]*R[[#, 2, 2]] - R[[#, 1, 2]]*R[[#, 2, 1]] == 
         R[[#, 3, 3]], 
        R[[#, 2, 3]]*R[[#, 3, 2]] - 
          R[[#, 2, 2]]*R[[#, 3, 3]] == -R[[#, 1, 1]], 
        R[[#, 2, 1]]*R[[#, 3, 3]] - 
          R[[#, 2, 3]]*R[[#, 3, 1]] == -R[[#, 1, 2]], 
        R[[#, 2, 2]]*R[[#, 3, 1]] - 
          R[[#, 2, 1]]*R[[#, 3, 2]] == -R[[#, 1, 3]], 
        R[[#, 1, 2]]*R[[#, 3, 3]] - 
          R[[#, 1, 3]]*R[[#, 3, 2]] == -R[[#, 2, 1]], 
        R[[#, 1, 3]]*R[[#, 3, 1]] - 
          R[[#, 1, 1]]*R[[#, 3, 3]] == -R[[#, 2, 2]], 
        R[[#, 1, 1]]*R[[#, 3, 2]] - 
          R[[#, 1, 2]]*R[[#, 3, 1]] == -R[[#, 2, 3]], 
        R[[#, 1, 3]]*R[[#, 2, 2]] - 
          R[[#, 1, 2]]*R[[#, 2, 3]] == -R[[#, 3, 1]], 
        R[[#, 1, 1]]*R[[#, 2, 3]] - 
          R[[#, 1, 3]]*R[[#, 2, 1]] == -R[[#, 3, 2]], 
        R[[#, 1, 2]]*R[[#, 2, 1]] - 
          R[[#, 1, 1]]*R[[#, 2, 2]] == -R[[#, 3, 3]]} &, 
      Range[1, Dimensions[R][[1]]]]];
   eq = Flatten[Append[eq, customeq], 1];
   Return[Refine[FullSimplify[Expression, eq],Assumptions:>eq]];
   );

SimplifyQ[Expression_, Q_, customeq_,Tconstr_:{0.5,300}] := (
eq=Table[{
	Q[[i,1]]^2+Q[[i,2]]^2+Q[[i,3]]^2+Q[[i,4]]^2 == 1,
	Q[[i,1]]\[Element]Reals,
	Q[[i,2]]\[Element]Reals,
	Q[[i,3]]\[Element]Reals,
	Q[[i,4]]\[Element]Reals,
	-1<=Q[[i,1]]<=1,
	-1<=Q[[i,2]]<=1,
	-1<=Q[[i,3]]<=1,
	-1<=Q[[i,4]]<=1},{i,1,Length[Q]}];   
   eq = Flatten[Append[eq, customeq], 1];
   Return[ParallelTable[Quiet[FullSimplify[Refine[Expression[[i]],eq],eq,TimeConstraint->Tconstr]],{i,1,Length[Expression]}]];
   );

SimplifyQLite[Expression_, Q_, customeq_,Tconstr_:{0.5,300}] := (
eq=Table[{
	Q[[i,1]]^2+Q[[i,2]]^2+Q[[i,3]]^2+Q[[i,4]]^2 == 1,
	Q[[i,1]]\[Element]Reals,
	Q[[i,2]]\[Element]Reals,
	Q[[i,3]]\[Element]Reals,
	Q[[i,4]]\[Element]Reals,
	-1<=Q[[i,1]]<=1,
	-1<=Q[[i,2]]<=1,
	-1<=Q[[i,3]]<=1,
	-1<=Q[[i,4]]<=1},{i,1,Length[Q]}];   
   eq = Flatten[Append[eq, customeq], 1];
   Return[ParallelTable[Quiet[Simplify[Refine[Expression[[i]],eq],eq,TimeConstraint->Tconstr]],{i,1,Length[Expression]}]];
   );

SimplifyQLiteMatrix[Expression_, Q_, customeq_,Tconstr_:{0.5,300}] := (
eq=Table[{
	Q[[i,1]]^2+Q[[i,2]]^2+Q[[i,3]]^2+Q[[i,4]]^2 == 1,
	Q[[i,1]]\[Element]Reals,
	Q[[i,2]]\[Element]Reals,
	Q[[i,3]]\[Element]Reals,
	Q[[i,4]]\[Element]Reals,
	-1<=Q[[i,1]]<=1,
	-1<=Q[[i,2]]<=1,
	-1<=Q[[i,3]]<=1,
	-1<=Q[[i,4]]<=1},{i,1,Length[Q]}];   
   eq = Flatten[Append[eq, customeq], 1];
   Return[ParallelTable[Quiet[Simplify[Refine[Expression[[i,j]],eq],eq,TimeConstraint->Tconstr]],{i,1,Dimensions[Expression][[1]]},{j,1,Dimensions[Expression][[2]]}]];
   );
AreReals[X_]:=(
vars = Flatten[X];
Return[Flatten[Table[vars[[i]]\[Element]Reals,{i,1,Length[vars]}]]]
);

