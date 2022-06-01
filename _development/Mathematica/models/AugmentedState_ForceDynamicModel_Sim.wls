#!/usr/bin/env wolframscript
(* ::Package:: *)

Clear["Global`*"]



parentPath = $InputFileName /. "" :> NotebookFileName[]
parentDir = DirectoryName @ parentPath
Import[parentDir<>"SimplifyFunctions.m"]
Import[parentDir<>"QuaternionAlgebra.m"]


(* ::Title:: *)
(*Useful Functions*)


L2[x_]:=Sqrt[Sum[x[[j]]^2,{j,1,Length[x]}]];
VecNorm[x_]:=Table[x[[i]]/L2[x],{i,1,Length[x]}];


SkewMatrix4[v_]:={{0,-v[[1]],-v[[2]],-v[[3]]},{v[[1]],0,v[[3]],-v[[2]]},{v[[2]],-v[[3]],0,v[[1]]},{v[[3]],v[[2]],-v[[1]],0}}/2;
SkewMatrix[v_]:={{0,-v[[3]],v[[2]]},{v[[3]],0,-v[[1]]},{-v[[2]],v[[1]],0}};


SubVec[exp_,v1_,v2_]:=exp/.Table[v1[[i]]->v2[[i]],{i,1,Dimensions[v1,1][[1]]}];
VecToZero[exp_,v_]:=exp/.Table[v[[i]]->0,{i,1,Dimensions[v,1][[1]]}];


ToGoodC[exp_]:=Module[{oexp}, oexp=Experimental`OptimizeExpression[exp];
If[Dimensions [StringPosition[ToString[InputForm[oexp]],"Compile"],1][[1]]>0,Block[{ locals, code},{locals,code}=ReleaseHold[(Hold@@oexp)/.Verbatim[Block][vars_,seq_]:>{vars,Hold[seq]}];ToString[CForm[code]]]
, ToString[CForm[exp]]]]
MyStringWrite[str_,file_]:=Module[{stream},stream=OpenWrite[file];WriteString[stream,str];Close[stream];]


(* ::Title:: *)
(*Beginning of stuff*)


(* ::Section:: *)
(*State*)


X=Table[x[i],{i,1,39}];
POSE=X[[{1,2,3}]];
Q=X[[{4,5,6,7}]];
V=X[[{8,9,10}]];
W=X[[{11,12,13}]];
A=X[[{14,15,16}]];
Alpha=X[[{17,18,19}]];
QWSprev = X[[30;;33]];
VSprev = X[[34;;36]];
WSprev = X[[37;;39]];


(* ::Section:: *)
(*Noises and Inputs*)


Z = Table[z[i],{i,1,6}];
NOISES=Table[eta[i],{i,1,6}];
ZNoise = Z + Table[NOISES[[i]],{i,1,6}];



(* ::PageBreak:: *)
(**)


(* ::Section:: *)
(*Parameters*)


(* Inertia *)
IBD={ibd[1],ibd[2],ibd[3]};(* diagonal terms *)
IBOD={ibod[1],ibod[2],ibod[3]};(* off diagonal terms*)

Ib = {{Exp[ibd[1]],ibod[1],ibod[2]},{ibod[1],Exp[ibd[2]],ibod[3]},{ibod[2],ibod[3],Exp[ibd[3]]}};
Ibinv=Inverse[Ib]//FullSimplify;



(* ::Section:: *)
(*Equations*)


AhatLong = ZNoise[[{1,2,3}]] + QuatToRot[QuatInv[Q]].{0,0,9.8066};  
Ahat = AhatLong;
Ahat
(*AlphaHatLong = Ibinv.( Ib.ZNoise[[{4,5,6}]] +  SkewMatrix[W].(Ib.W));*)
AlphaHatLong = ZNoise[[{4,5,6}]];
AlphaHat = AlphaHatLong;


(* ::Subsection:: *)
(*Errors*)


ERR = Flatten[{A-Ahat,Alpha-AlphaHat}];


(* ::Section:: *)
(*Output*)


SetDirectory[parentDir];
Reg={Sign'[_]->0,Abs'[x_]->Sign[x]};


(* ::Subsection:: *)
(*Jacobians*)


(* ::Subsubsection:: *)
(*State*)


Err = VecToZero[ERR,NOISES];
JErrNoises=VecToZero[D[ERR,{NOISES}]/.Reg,NOISES];
JErrQ=VecToZero[D[ERR,{Q}]/.Reg,NOISES];
JErrV=VecToZero[D[ERR,{V}]/.Reg,NOISES];
JErrW=VecToZero[D[ERR,{W}]/.Reg,NOISES];
JErrA=VecToZero[D[ERR,{A}]/.Reg,NOISES];
JErrAlpha=VecToZero[D[ERR,{Alpha}]/.Reg,NOISES];
JErrQprev=VecToZero[D[ERR,{QWSprev}]/.Reg,NOISES];
JErrVprev=VecToZero[D[ERR,{VSprev}]/.Reg,NOISES];
JErrWprev=VecToZero[D[ERR,{WSprev}]/.Reg,NOISES];


(* ::Subsubsection:: *)
(*Parameters*)


JErrIBD=VecToZero[D[ERR,{IBD}]/.Reg,NOISES];
JErrIBOD=VecToZero[D[ERR,{IBOD}]/.Reg,NOISES];


(* ::Subsubsection:: *)
(*2nd order derivatives*)


AOut = VecToZero[AhatLong,NOISES];
AlphaOut = VecToZero[AlphaHatLong,NOISES];


(* ::Subsection:: *)
(*Mthout files*)


Run["rm *.cppready"];


(* ::Subsubsection:: *)
(*State*)


MyStringWrite[ToGoodC[Err],"ForceDynamicModel_Err.mthout"];
MyStringWrite[ToGoodC[JErrNoises],"ForceDynamicModel_JErrNoises.mthout"];
MyStringWrite[ToGoodC[JErrQ],"ForceDynamicModel_JErrQ.mthout"];
MyStringWrite[ToGoodC[JErrV],"ForceDynamicModel_JErrV.mthout"];
MyStringWrite[ToGoodC[JErrW],"ForceDynamicModel_JErrW.mthout"];
MyStringWrite[ToGoodC[JErrA],"ForceDynamicModel_JErrA.mthout"];
MyStringWrite[ToGoodC[JErrAlpha],"ForceDynamicModel_JErrAlpha.mthout"];
MyStringWrite[ToGoodC[JErrQprev],"ForceDynamicModel_JErrQprev.mthout"];
MyStringWrite[ToGoodC[JErrVprev],"ForceDynamicModel_JErrVprev.mthout"];
MyStringWrite[ToGoodC[JErrWprev],"ForceDynamicModel_JErrWprev.mthout"];


(* ::Subsubsection:: *)
(*Parameters*)


MyStringWrite[ToGoodC[JErrIBD],"ForceDynamicModel_JErrIBD.mthout"];
MyStringWrite[ToGoodC[JErrIBOD],"ForceDynamicModel_JErrIBOD.mthout"];


(* ::Subsubsection:: *)
(*2nd order derivatives*)


MyStringWrite[ToGoodC[AOut],"ForceDynamicModel_AOut.mthout"];
MyStringWrite[ToGoodC[AlphaOut],"ForceDynamicModel_AlphaOut.mthout"];


(* ::Subsection:: *)
(*FixMathematicaOutput*)


(* ::Subsubsection:: *)
(*State*)


Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_Err.mthout err 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrNoises.mthout J 0 0"];

Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrQ.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrV.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrW.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrA.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrAlpha.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrQprev.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrVprev.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrWprev.mthout J 0 0"];


Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrIBD.mthout J 0 0"];
Run["python ../fixMathematicaOutput_v2.py ForceDynamicModel_JErrIBOD.mthout J 0 0"];


(* ::PageBreak:: *)
(**)


(* ::Subsection:: *)
(*Final Steps*)


Run["mv *.cppready ../../../roamfree/ROAMfunctions/generated"];
Run["rm *.cppready"];
Run["rm *.mthout"];
